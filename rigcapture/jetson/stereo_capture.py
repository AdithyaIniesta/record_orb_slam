#!/usr/bin/env python3
"""
stereo_capture.py - record a synchronised stereo dataset on the Jetson while
streaming a live focus-check preview to the PC.

Capture front end is the same as JetsonTracker (src/dual/pipeline.cpp):
    v4l2src (UYVY 1280x720@60) -> nvvidconv (VIC) -> frames in system memory
Streaming back end is the same too:
    H.264 (nvv4l2h264enc) -> rtph264pay -> UDP, ports 5000 / 5002,
so the README's `gst-launch-1.0 udpsrc port=5000 ...` viewer works unchanged.

Run ON THE JETSON, with JetsonTracker stopped (two programs cannot open the
same camera):

    python3 stereo_capture.py --dest <PC ip>        # preview only
    python3 stereo_capture.py --dest <PC ip> --autostart --duration 60

Keys (in the terminal that runs it):  r = start/stop recording   f = reset focus peak   q = quit

Dataset layout (EuRoC "mav0" style, readable by ORB-SLAM3's *_euroc examples; the IMU of a later
Cube Orange recording goes into mav0/imu0 of the same folder):

    <out>/stereo_0001/                 (mono: only cam0)
        meta.json            cameras, resolution, rates, clock anchors, counts
        times.txt            one timestamp (ns) per frame / pair - ORB-SLAM3 timestamp file
        pairs.csv            (two cameras only) ts_ns,left_mono_ns,right_mono_ns,skew_ns,left_seq,right_seq
        mav0/cam0/data/<ts>.png    cam0 = boresight (/dev/video0 here), 8-bit gray
        mav0/cam0/data.csv         #timestamp [ns],filename
        mav0/cam1/...              (two cameras only) same file names as cam0

Timestamps are CLOCK_MONOTONIC nanoseconds taken from the V4L2 buffer time.
Device wall clocks are not trustworthy on these Jetsons (see CLAUDE.md), so
nothing here orders anything by wall-clock time; meta.json stores the clock
anchors only for reference.
"""
import argparse
import collections
import glob
import json
import os
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

try:
    import cv2
    import numpy as np
except ImportError as e:  # pragma: no cover
    sys.exit("stereo_capture: needs numpy and OpenCV (python3-opencv): %s" % e)
try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except Exception as e:  # pragma: no cover
    sys.exit("stereo_capture: needs PyGObject + GStreamer (sudo apt install python3-gi "
             "gir1.2-gstreamer-1.0): %s" % e)

Gst.init(None)

NS = 1000000000
API_VERSION = 5      # bumped when the status/command interface changes; the PC app upgrades older ones


# ----------------------------------------------------------------------------
# GStreamer pipeline strings
# ----------------------------------------------------------------------------
def have_element(name):
    return Gst.ElementFactory.find(name) is not None


def capture_pipeline_str(dev, fmt, w, h, fps, sim_index=None):
    """Camera -> NV12 frames in system memory -> appsink 'sink'."""
    sink = "appsink name=sink emit-signals=true sync=false max-buffers=8 drop=true"
    if sim_index is not None:
        pattern = ("ball", "smpte")[sim_index % 2]
        return ("videotestsrc is-live=true pattern=%s ! "
                "video/x-raw,format=I420,width=%d,height=%d,framerate=%d/1 ! "
                "videoconvert ! video/x-raw,format=NV12 ! %s" % (pattern, w, h, fps, sink))
    fmt = fmt.upper()
    if fmt in ("UYVY", "YUYV", "YUY2"):
        raw = "UYVY" if fmt == "UYVY" else "YUY2"
        src = ("v4l2src device=%s ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
               % (dev, raw, w, h, fps))
        if have_element("nvvidconv"):
            # VIC does the colour conversion, same as the tracker (a CPU
            # videoconvert at 720p caps the pipeline at ~59 fps).
            return src + "nvvidconv ! video/x-raw,format=NV12 ! " + sink
        return src + "videoconvert ! video/x-raw,format=NV12 ! " + sink
    # MJPG fallback path, same decoder as the tracker
    return ("v4l2src device=%s ! image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=NV12 ! %s" % (dev, w, h, fps, sink))


def stream_pipeline_str(w, h, fps, dest, port):
    """appsrc (BGR preview frames) -> H.264 -> RTP -> UDP. Mirrors build_stream_pipeline()."""
    gop = max(1, fps // 2)
    if have_element("nvv4l2h264enc"):
        enc = ("videoconvert ! video/x-raw,format=NV12 ! nvvidconv ! "
               "video/x-raw(memory:NVMM),format=NV12 ! "
               "queue max-size-buffers=2 leaky=downstream ! "
               "nvv4l2h264enc name=venc insert-sps-pps=true iframeinterval=%d idrinterval=%d ! "
               "h264parse config-interval=1" % (gop, gop))
    else:  # PC / test path
        enc = ("videoconvert ! video/x-raw,format=I420 ! "
               "x264enc name=venc tune=zerolatency speed-preset=ultrafast key-int-max=%d ! "
               "h264parse config-interval=1" % gop)
    return ("appsrc name=src is-live=true format=time do-timestamp=true block=false ! "
            "video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
            "queue max-size-buffers=2 leaky=downstream ! %s ! "
            "rtph264pay mtu=1400 config-interval=1 pt=96 ! "
            "udpsink host=%s port=%d sync=false" % (w, h, fps, enc, dest, port))


# ----------------------------------------------------------------------------
# One camera
# ----------------------------------------------------------------------------
class Frame(object):
    __slots__ = ("seq", "t_ns", "arrival_ns", "y")

    def __init__(self, seq, t_ns, arrival_ns, y):
        self.seq, self.t_ns, self.arrival_ns, self.y = seq, t_ns, arrival_ns, y


class Camera(object):
    QUEUE_LEN = 16

    def __init__(self, index, role, device, args):
        self.index, self.role, self.device, self.args = index, role, device, args
        self.name = "cam%d" % index
        self.pipe = None
        self.base_time = None
        self.queue = collections.deque(maxlen=self.QUEUE_LEN)
        self.latest = None
        self.seq = 0
        self.arrivals = collections.deque(maxlen=120)
        self.q_drops = 0
        self.last_frame_wall = 0.0
        self.started_wall = 0.0
        self.restarts = 0
        self.stride_warned = False
        self.clock_warned = False
        self.metrics = (0.0, 0.0, 0.0, 0.0, 0.0)   # focus, mean, % clipped, focus peak, focus/peak (preview thread)
        self.lock = threading.Lock()

    # -- lifecycle ----------------------------------------------------------
    def start(self):
        a = self.args
        desc = capture_pipeline_str(self.device, a.format, a.width, a.height, a.fps,
                                    self.index if a.sim else None)
        if os.environ.get("STEREO_VERBOSE"):
            print("[%s] %s" % (self.name, desc))
        self.pipe = Gst.parse_launch(desc)
        self.pipe.get_by_name("sink").connect("new-sample", self._on_sample)
        self.base_time = None
        self.started_wall = time.time()
        self.pipe.set_state(Gst.State.PLAYING)
        ret, _, _ = self.pipe.get_state(5 * Gst.SECOND)
        if ret == Gst.StateChangeReturn.FAILURE:
            msg = self.poll_bus() or "no message"
            self.stop()
            raise RuntimeError("%s (%s) failed to start: %s" % (self.name, self.device, msg))

    def stop(self):
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)
            # wait so the kernel really releases the V4L2 device before a rebuild
            self.pipe.get_state(2 * Gst.SECOND)
            self.pipe = None

    def poll_bus(self):
        if self.pipe is None:
            return None
        bus = self.pipe.get_bus()
        out = None
        while True:
            m = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if m is None:
                break
            if m.type == Gst.MessageType.ERROR:
                err, dbg = m.parse_error()
                out = "%s: %s" % (m.src.get_name() if m.src else "?", err.message)
            else:
                out = "end of stream"
        return out

    # -- frame callback (GStreamer streaming thread; keep it short) ------------
    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        arrival = time.monotonic_ns()
        a = self.args
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            size = mi.size
            stride = (size * 2) // (3 * a.height)       # NV12/I420: Y plane first, 1.5 planes total
            if stride < a.width or (stride * a.height * 3) // 2 != size:
                if not self.stride_warned:
                    print("[%s] unexpected frame size %d for %dx%d, assuming no padding"
                          % (self.name, size, a.width, a.height))
                    self.stride_warned = True
                stride = a.width
            y = np.frombuffer(mi.data, np.uint8, count=stride * a.height).copy()
        finally:
            buf.unmap(mi)
        y = y.reshape(a.height, stride)
        if stride != a.width:
            y = np.ascontiguousarray(y[:, :a.width])

        # Frame time = V4L2 buffer time on CLOCK_MONOTONIC (running time + base time).
        if self.base_time is None and self.pipe is not None:
            self.base_time = self.pipe.get_base_time()
        t = arrival
        if buf.pts != Gst.CLOCK_TIME_NONE and self.base_time is not None:
            cand = self.base_time + buf.pts
            if abs(cand - arrival) < NS:
                t = cand
            elif not self.clock_warned:
                print("[%s] buffer time is %.1f s off the monotonic clock; using arrival time"
                      % (self.name, (cand - arrival) / 1e9))
                self.clock_warned = True

        self.seq += 1
        f = Frame(self.seq, t, arrival, y)
        if len(self.queue) == self.queue.maxlen:
            self.q_drops += 1
        self.queue.append(f)
        self.latest = f
        self.arrivals.append(arrival)
        self.last_frame_wall = time.time()
        return Gst.FlowReturn.OK

    def fps(self):
        arr = list(self.arrivals)
        if len(arr) < 5:
            return 0.0
        return (len(arr) - 1) * 1e9 / float(arr[-1] - arr[0])


# ----------------------------------------------------------------------------
# Pairing: match left/right frames by V4L2 timestamp
# ----------------------------------------------------------------------------
class Pairer(threading.Thread):
    """Greedy nearest-in-time matching. Tolerance is half a frame period, so every
    frame has at most one partner; a frame with no partner is counted and dropped."""

    def __init__(self, left, right, fps, on_pair, tol_ms=None):
        threading.Thread.__init__(self, name="pairer", daemon=True)
        self.left, self.right, self.on_pair = left, right, on_pair
        self.tol_ns = int((tol_ms if tol_ms else 500.0 / fps) * 1e6)
        self.running = True
        self.unpaired = [0, 0]
        self.pairs = 0
        self.skews = collections.deque(maxlen=300)

    def run(self):
        ql, qr = self.left.queue, self.right.queue
        while self.running:
            try:
                if not ql or not qr:
                    time.sleep(0.001)
                    continue
                l, r = ql[0], qr[0]
                d = r.t_ns - l.t_ns
                if abs(d) <= self.tol_ns:
                    ql.popleft()
                    qr.popleft()
                    self.pairs += 1
                    self.skews.append(d)
                    self.on_pair(l, r, d)
                elif d < 0:
                    qr.popleft()
                    self.unpaired[1] += 1
                else:
                    ql.popleft()
                    self.unpaired[0] += 1
            except IndexError:      # a queue emptied between the check and the pop
                pass

    def skew_stats_ms(self):
        s = [abs(x) for x in list(self.skews)]
        if not s:
            return 0.0, 0.0
        return sum(s) / len(s) / 1e6, max(s) / 1e6


class Feeder(threading.Thread):
    """Mono mode: hands every frame of the one camera to the recorder (same interface as Pairer)."""

    def __init__(self, cam, on_frame):
        threading.Thread.__init__(self, name="feeder", daemon=True)
        self.cam, self.on_frame = cam, on_frame
        self.running = True
        self.pairs = 0
        self.unpaired = [0, 0]

    def run(self):
        q = self.cam.queue
        while self.running:
            try:
                f = q.popleft()
            except IndexError:
                time.sleep(0.001)
                continue
            self.pairs += 1
            self.on_frame(f, None, 0)

    def skew_stats_ms(self):
        return 0.0, 0.0


# ----------------------------------------------------------------------------
# Dataset writer
# ----------------------------------------------------------------------------
def next_dataset_dir(out_root, name=None):
    """stereo_0001, stereo_0002, ... A counter, not a date: the device clock is unreliable."""
    os.makedirs(out_root, exist_ok=True)
    if name:
        d = os.path.join(out_root, name)
        if os.path.exists(d):
            raise RuntimeError("%s already exists" % d)
        return d
    n = 1
    while os.path.exists(os.path.join(out_root, "stereo_%04d" % n)):
        n += 1
    return os.path.join(out_root, "stereo_%04d" % n)


def read_rtc_epoch():
    try:
        with open("/sys/class/rtc/rtc0/since_epoch") as fh:
            return int(fh.read().strip())
    except Exception:
        return None


class DatasetWriter(object):
    MAX_INFLIGHT = 24

    def __init__(self, root, args, cams):
        self.root, self.args, self.cams = root, args, cams
        self.stereo = len(cams) == 2
        self.names = ["cam%d" % i for i in range(len(cams))]
        self.dirs = {}
        for n in self.names:
            self.dirs[n] = os.path.join(root, "mav0", n, "data")
            os.makedirs(self.dirs[n])
        self.rows = dict((n, []) for n in self.names)
        self.pair_rows = []
        self.lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=args.writers)
        self.inflight = 0
        self.written = 0
        self.write_drops = 0
        self.errors = 0
        self.bytes = 0
        self.t_first = None
        self.t_last = None
        self.last_ts = None
        self.started = time.time()
        self.params = [cv2.IMWRITE_PNG_COMPRESSION, args.png_level]
        self.clock_anchor = {"monotonic_ns": time.monotonic_ns(), "realtime_ns": time.time_ns(),
                             "rtc_since_epoch_s": read_rtc_epoch(),
                             "note": "device clocks are unreliable; use monotonic timestamps"}
        self._write_meta(final=False)

    def want(self, t_ns):
        """Decimate to --record-fps by time, so a dropped frame does not shift the cadence."""
        if self.last_ts is None or t_ns - self.last_ts >= int(NS / self.args.record_fps) - int(2e6):
            return True
        return False

    def submit(self, l, r, skew):
        with self.lock:
            if self.inflight >= self.MAX_INFLIGHT:
                self.write_drops += 1
                return
            self.inflight += 1
        self.last_ts = l.t_ns
        self.pool.submit(self._write, l, r, skew)

    def _write(self, l, r, skew):
        try:
            name = "%d.png" % l.t_ns
            p0 = os.path.join(self.dirs["cam0"], name)
            ok = cv2.imwrite(p0, l.y, self.params)
            size = os.path.getsize(p0) if ok else 0
            if r is not None:
                p1 = os.path.join(self.dirs["cam1"], name)
                ok = cv2.imwrite(p1, r.y, self.params) and ok
                size += os.path.getsize(p1) if os.path.exists(p1) else 0
            if not ok:
                raise IOError("imwrite failed")
            with self.lock:
                self.rows["cam0"].append((l.t_ns, name))
                if r is not None:
                    self.rows["cam1"].append((r.t_ns, name))
                    self.pair_rows.append((l.t_ns, l.t_ns, r.t_ns, skew, l.seq, r.seq))
                self.written += 1
                self.bytes += size
                self.t_first = l.t_ns if self.t_first is None else self.t_first
                self.t_last = l.t_ns
        except Exception as e:   # disk full, unplugged SSD, ...
            with self.lock:
                self.errors += 1
            print("[writer] %s" % e)
        finally:
            with self.lock:
                self.inflight -= 1

    def _write_meta(self, final):
        a = self.args
        meta = {
            "format": "EuRoC mav0 layout, 8-bit gray PNG, %s" % ("stereo" if self.stereo else "mono"),
            "cameras": len(self.cams),
            "width": a.width, "height": a.height, "capture_fps": a.fps, "pixel_format": a.format,
            "record_fps": a.record_fps, "png_compression": a.png_level,
            "timestamps": "CLOCK_MONOTONIC ns (V4L2 buffer time)",
            "clock_anchor": self.clock_anchor,
            "imu": getattr(self, "imu_info", None) or "not recorded",
            "note": a.note, "argv": sys.argv, "sim": a.sim,
            "complete": final,
        }
        for i, c in enumerate(self.cams):
            meta["cam%d" % i] = {"role": c.role, "device": c.device, "device_resolved": os.path.realpath(c.device)}
        if self.stereo:
            meta["pairing"] = ("nearest V4L2 timestamp within half a frame period; cam1 file carries cam0's "
                               "timestamp in its name, real time is in pairs.csv")
        if final:
            dur = ((self.t_last - self.t_first) / 1e9) if self.written > 1 else 0.0
            meta.update({"pairs_written": self.written, "frames_written": self.written,
                         "write_drops": self.write_drops, "write_errors": self.errors, "bytes": self.bytes,
                         "duration_s": round(dur, 3),
                         "measured_rate_hz": round((self.written - 1) / dur, 3) if dur > 0 else 0})
        with open(os.path.join(self.root, "meta.json"), "w") as fh:
            json.dump(meta, fh, indent=2)

    def finish(self):
        self.pool.shutdown(wait=True)
        with self.lock:
            for cam in self.names:
                rows = sorted(self.rows[cam])
                with open(os.path.join(self.root, "mav0", cam, "data.csv"), "w") as fh:
                    fh.write("#timestamp [ns],filename\n")
                    for t, n in rows:
                        fh.write("%d,%s\n" % (t, n))
            if self.stereo:
                pr = sorted(self.pair_rows)
                with open(os.path.join(self.root, "pairs.csv"), "w") as fh:
                    fh.write("ts_ns,left_mono_ns,right_mono_ns,skew_ns,left_seq,right_seq\n")
                    for row in pr:
                        fh.write("%d,%d,%d,%d,%d,%d\n" % row)
                stamps = [row[0] for row in pr]
            else:
                stamps = [t for t, _ in sorted(self.rows["cam0"])]
            with open(os.path.join(self.root, "times.txt"), "w") as fh:
                for t in stamps:
                    fh.write("%d\n" % t)
        self._write_meta(final=True)
        return self.written


# ----------------------------------------------------------------------------
# Focus / exposure metrics and preview HUD
# ----------------------------------------------------------------------------
def focus_score(y):
    """Variance of the Laplacian over the central half of the frame, at full resolution.
    Only comparable between frames of the SAME scene at the same distance and lighting."""
    h, w = y.shape
    roi = y[h // 4:3 * h // 4, w // 4:3 * w // 4]
    return float(cv2.Laplacian(roi, cv2.CV_32F, ksize=3).var())


def exposure_stats(y):
    sub = y[::4, ::4]
    return float(sub.mean()), 100.0 * float(((sub >= 250).sum() + (sub <= 5).sum())) / sub.size


def put_text(img, text, org, color=(255, 255, 255), scale=0.5):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


class Preview(threading.Thread):
    """Builds the focus-check picture for one camera and pushes it into its H.264 stream."""

    def __init__(self, cam, args, app):
        threading.Thread.__init__(self, name="preview-" + cam.name, daemon=True)
        self.cam, self.args, self.app = cam, args, app
        self.pw = args.preview_width
        self.ph = (args.preview_width * args.height // args.width) & ~1
        self.running = True
        self.scores = collections.deque()
        self.pipe = None
        self.src = None
        self.last_img = None
        self.scores.clear()
        if not args.no_stream:
            port = args.left_port if cam.index == 0 else args.right_port
            self.pipe = Gst.parse_launch(stream_pipeline_str(self.pw, self.ph, args.preview_fps,
                                                             args.dest, port))
            self.src = self.pipe.get_by_name("src")
            venc = self.pipe.get_by_name("venc")
            if venc is not None and venc.find_property("bitrate") is not None:
                venc.set_property("bitrate", args.stream_kbps * 1000)
            self.pipe.set_state(Gst.State.PLAYING)

    def render(self):
        cam, a = self.cam, self.args
        f = cam.latest
        stale = f is None or time.time() - cam.last_frame_wall > 1.0
        img = np.zeros((self.ph, self.pw, 3), np.uint8)
        label = "%s %s" % (cam.name, cam.role)
        if stale:
            put_text(img, label, (10, 22))
            put_text(img, "NO SIGNAL  (restarts: %d)" % cam.restarts, (10, self.ph // 2), (0, 0, 255), 0.8)
            return img
        y = f.y
        if not a.hud:
            # Clean picture for the PC app: it draws its own record indicator, and the operator judges focus by eye.
            cam.metrics = (0.0,) + exposure_stats(y) + (0.0, 0.0)
            return cv2.cvtColor(cv2.resize(y, (self.pw, self.ph), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        score = focus_score(y)
        now = time.time()
        self.scores.append((now, score))
        while self.scores and now - self.scores[0][0] > a.peak_window:
            self.scores.popleft()
        peak = max(s for _, s in self.scores)
        ratio = score / peak if peak > 0 else 0.0
        mean, clip = exposure_stats(y)
        cam.metrics = (score, mean, clip, peak, ratio)

        small = cv2.resize(y, (self.pw, self.ph), interpolation=cv2.INTER_AREA)
        img = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)

        # focus ROI box and 2x magnified 1:1 inset of the centre
        h, w = y.shape
        sx, sy = self.pw / float(w), self.ph / float(h)
        cv2.rectangle(img, (int(w // 4 * sx), int(h // 4 * sy)),
                      (int(3 * w // 4 * sx), int(3 * h // 4 * sy)), (255, 200, 0), 1)
        iw, ih = 160, 90
        crop = y[h // 2 - ih // 2:h // 2 + ih // 2, w // 2 - iw // 2:w // 2 + iw // 2]
        inset = cv2.cvtColor(cv2.resize(crop, (iw * 2, ih * 2), interpolation=cv2.INTER_NEAREST),
                             cv2.COLOR_GRAY2BGR)
        x0, y0 = self.pw - inset.shape[1] - 6, self.ph - inset.shape[0] - 6
        if x0 > 0 and y0 > 0:
            img[y0:y0 + inset.shape[0], x0:x0 + inset.shape[1]] = inset
            cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + inset.shape[1], y0 + inset.shape[0]), (255, 255, 255), 1)
            put_text(img, "centre x2", (x0 + 4, y0 + 14), scale=0.4)

        col = (0, 200, 0) if ratio >= 0.9 else ((0, 200, 255) if ratio >= 0.7 else (0, 0, 255))
        put_text(img, "%s  %dx%d @%.1f fps" % (label, w, h, cam.fps()), (10, 22))
        put_text(img, "FOCUS %.0f   peak(%ds) %.0f   %d%%" % (score, a.peak_window, peak, int(ratio * 100)),
                 (10, 44), col)
        cv2.rectangle(img, (10, 52), (10 + int(180 * min(ratio, 1.0)), 60), col, -1)
        cv2.rectangle(img, (10, 52), (190, 60), (255, 255, 255), 1)
        put_text(img, "mean %.0f   clipped %.1f%%" % (mean, clip), (10, 80),
                 (0, 0, 255) if clip > 2.0 else (255, 255, 255))
        self.app.draw_status(img, cam)
        return img

    def run(self):
        period = 1.0 / self.args.preview_fps
        nxt = time.time()
        while self.running:
            nxt += period
            try:
                img = self.render()
                self.last_img = img
                if self.src is not None:
                    self.src.emit("push-buffer", Gst.Buffer.new_wrapped(img.tobytes()))
            except Exception as e:
                print("[preview %s] %s" % (self.cam.name, e))
            dt = nxt - time.time()
            if dt > 0:
                time.sleep(dt)
            else:
                nxt = time.time()

    def stop(self):
        self.running = False
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)



# ----------------------------------------------------------------------------
# IMU from a Cube Orange (ArduPilot) over MAVLink - Python standard library only
# (no pymavlink / pyserial needed on the Jetson). Frames are verified with the CRC byte of every message ArduPilot
# can send, so the stream re-synchronises by itself after any garbage.
# ----------------------------------------------------------------------------
MAVLINK_CRC_EXTRA = {
    0: 50, 1: 124, 2: 137, 4: 237, 5: 217, 6: 104, 7: 119, 11: 89,
    20: 214, 21: 159, 22: 220, 23: 168, 24: 24, 25: 23, 26: 170, 27: 144,
    28: 67, 29: 115, 30: 39, 31: 246, 32: 185, 33: 104, 34: 237, 35: 244,
    36: 222, 37: 212, 38: 9, 39: 254, 40: 230, 41: 28, 42: 28, 43: 132,
    44: 221, 45: 232, 46: 11, 47: 153, 48: 41, 49: 39, 50: 78, 51: 196,
    54: 15, 55: 3, 61: 167, 62: 183, 63: 119, 64: 191, 65: 118, 66: 148,
    67: 21, 69: 243, 70: 124, 73: 38, 74: 20, 75: 158, 76: 152, 77: 143,
    81: 106, 82: 49, 83: 22, 84: 143, 85: 140, 86: 5, 87: 150, 89: 231,
    90: 183, 91: 63, 92: 54, 93: 47, 100: 175, 101: 102, 102: 158, 103: 208,
    104: 56, 105: 93, 106: 138, 107: 108, 108: 32, 109: 185, 110: 84, 111: 34,
    112: 174, 113: 124, 114: 237, 115: 4, 116: 76, 117: 128, 118: 56, 119: 116,
    120: 134, 121: 237, 122: 203, 123: 250, 124: 87, 125: 203, 126: 220, 127: 25,
    128: 226, 129: 46, 130: 29, 131: 223, 132: 85, 133: 6, 134: 229, 135: 203,
    136: 1, 137: 195, 138: 109, 139: 168, 140: 181, 141: 47, 142: 72, 143: 131,
    144: 127, 146: 103, 147: 154, 148: 178, 149: 200, 150: 134, 151: 219, 152: 208,
    153: 188, 154: 84, 155: 22, 156: 19, 157: 21, 158: 134, 160: 78, 161: 68,
    162: 189, 163: 127, 164: 154, 165: 21, 166: 21, 167: 144, 168: 1, 169: 234,
    170: 73, 171: 181, 172: 22, 173: 83, 174: 167, 175: 138, 176: 234, 177: 240,
    178: 47, 179: 189, 180: 52, 181: 174, 182: 229, 183: 85, 184: 159, 185: 186,
    186: 72, 191: 92, 192: 36, 193: 71, 194: 98, 195: 120, 200: 134, 201: 205,
    214: 69, 215: 101, 216: 50, 217: 202, 218: 17, 219: 162, 225: 208, 226: 207,
    230: 163, 231: 105, 232: 151, 233: 35, 234: 150, 235: 179, 241: 90, 242: 104,
    243: 85, 244: 95, 245: 130, 246: 184, 247: 81, 248: 8, 249: 204, 250: 49,
    251: 170, 252: 44, 253: 83, 254: 46, 256: 71, 257: 131, 258: 187, 259: 92,
    260: 146, 261: 179, 262: 12, 263: 133, 264: 49, 265: 26, 266: 193, 267: 35,
    268: 14, 269: 109, 270: 59, 271: 22, 275: 126, 276: 18, 277: 62, 280: 70,
    281: 48, 282: 123, 283: 74, 284: 99, 285: 137, 286: 210, 287: 1, 288: 20,
    295: 234, 296: 158, 299: 19, 301: 243, 310: 28, 311: 95, 320: 243, 321: 88,
    322: 243, 323: 78, 324: 132, 330: 23, 331: 91, 332: 236, 333: 231, 335: 225,
    339: 199, 340: 99, 345: 209, 350: 232, 360: 11, 370: 75, 373: 117, 375: 251,
    376: 199, 385: 147, 386: 132, 387: 4, 388: 8, 390: 156, 9000: 113, 9005: 117,
    10001: 209, 10002: 186, 10003: 4, 10004: 133, 10005: 103, 10006: 193, 10007: 71, 10008: 240,
    10151: 195, 11000: 134, 11001: 15, 11002: 234, 11003: 64, 11004: 11, 11005: 93, 11010: 46,
    11011: 106, 11020: 205, 11030: 144, 11031: 133, 11032: 85, 11033: 195, 11034: 79, 11035: 128,
    11036: 177, 11037: 130, 11038: 47, 11039: 142, 11040: 132, 11041: 208, 11042: 201, 11043: 193,
    11044: 189, 11060: 162, 12900: 114, 12901: 254, 12902: 140, 12903: 249, 12904: 77, 12905: 49,
    12915: 94, 12918: 139, 12919: 7, 12920: 20, 42000: 227, 42001: 239, 50001: 246, 50002: 181,
    50003: 62, 50004: 240, 50005: 152, 52000: 13, 52001: 239,
}

# id: (name, struct format, field names in wire order) - taken from pymavlink's ardupilotmega dialect
MAVLINK_MSGS = {
    0: ('HEARTBEAT', '<IBBBBB', ('custom_mode', 'type', 'autopilot', 'base_mode', 'system_status', 'mavlink_version')),
    27: ('RAW_IMU', '<QhhhhhhhhhBh', ('time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'id', 'temperature')),
    30: ('ATTITUDE', '<Iffffff', ('time_boot_ms', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed')),
    76: ('COMMAND_LONG', '<fffffffHBBB', ('param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7', 'command', 'target_system', 'target_component', 'confirmation')),
    77: ('COMMAND_ACK', '<HBBiBB', ('command', 'result', 'progress', 'result_param2', 'target_system', 'target_component')),
    111: ('TIMESYNC', '<qq', ('tc1', 'ts1')),
    116: ('SCALED_IMU2', '<Ihhhhhhhhhh', ('time_boot_ms', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'temperature')),
    129: ('SCALED_IMU3', '<Ihhhhhhhhhh', ('time_boot_ms', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag', 'temperature')),
}
MAVLINK_SIZE = dict((k, struct.calcsize(v[1])) for k, v in MAVLINK_MSGS.items())
MAV_CMD_SET_MESSAGE_INTERVAL = 511
G_MSS = 9.80665
IMU_STREAMS = (("RAW_IMU", "imu0", 27), ("SCALED_IMU2", "imu1", 116), ("SCALED_IMU3", "imu2", 129))
ATTITUDE_ID = 30
ACK_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavParser(object):
    """Incremental MAVLink 1/2 frame parser. feed(bytes) -> [(msgid, sysid, compid, payload_bytes)] for valid frames."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        buf = self.buf
        buf.extend(data)
        out = []
        pos = 0
        n = len(buf)
        while pos < n:
            b = buf[pos]
            if b == 0xFD:                                   # MAVLink 2
                if n - pos < 12:
                    break
                plen = buf[pos + 1]
                total = 10 + plen + 2 + (13 if buf[pos + 2] & 1 else 0)   # incompat flag 1 = signed frame
                if n - pos < total:
                    break
                msgid = buf[pos + 7] | (buf[pos + 8] << 8) | (buf[pos + 9] << 16)
                extra = MAVLINK_CRC_EXTRA.get(msgid)
                crc_rx = buf[pos + 10 + plen] | (buf[pos + 11 + plen] << 8)
                if extra is not None and x25(bytes(buf[pos + 1:pos + 10 + plen]) + bytes([extra])) == crc_rx:
                    out.append((msgid, buf[pos + 5], buf[pos + 6], bytes(buf[pos + 10:pos + 10 + plen])))
                    pos += total
                    continue
                self.bad += 1
                pos += 1
            elif b == 0xFE:                                 # MAVLink 1
                if n - pos < 8:
                    break
                plen = buf[pos + 1]
                total = 6 + plen + 2
                if n - pos < total:
                    break
                msgid = buf[pos + 5]
                extra = MAVLINK_CRC_EXTRA.get(msgid)
                crc_rx = buf[pos + 6 + plen] | (buf[pos + 7 + plen] << 8)
                if extra is not None and x25(bytes(buf[pos + 1:pos + 6 + plen]) + bytes([extra])) == crc_rx:
                    out.append((msgid, buf[pos + 3], buf[pos + 4], bytes(buf[pos + 6:pos + 6 + plen])))
                    pos += total
                    continue
                self.bad += 1
                pos += 1
            else:
                pos += 1
        del buf[:pos]
        if len(buf) > 4096:                                 # never let garbage pile up
            del buf[:-512]
        return out


def mav_decode(msgid, payload):
    """payload -> dict of fields for the messages in MAVLINK_MSGS (MAVLink 2 zero-trims the payload, so pad it)."""
    spec = MAVLINK_MSGS.get(msgid)
    if spec is None:
        return None
    size = MAVLINK_SIZE[msgid]
    p = payload[:size].ljust(size, b"\0")
    return dict(zip(spec[2], struct.unpack(spec[1], p)))


def mav_pack2(msgid, payload, seq, sysid=255, compid=191):
    """MAVLink 2 frame (zero-trimmed payload, unsigned). Defaults: system 255, component 191 (onboard computer)."""
    p = bytes(payload).rstrip(b"\0") or bytes(payload[:1])
    hdr = struct.pack("<BBBBBB", len(p), 0, 0, seq & 0xFF, sysid, compid) + struct.pack("<I", msgid)[:3]
    crc = x25(hdr + p + bytes([MAVLINK_CRC_EXTRA[msgid]]))
    return b"\xFD" + hdr + p + struct.pack("<H", crc)


class SerialPort(object):
    """Raw serial port with the standard library (works for USB CDC-ACM; the baud rate is irrelevant there)."""

    def __init__(self, dev, baud=115200):
        import termios
        import tty
        self.dev = dev
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            speed = getattr(termios, "B%d" % baud, termios.B115200)
            attrs[4] = attrs[5] = speed
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            os.close(self.fd)
            raise

    def read(self, timeout=0.05):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return b""
        if not data:
            raise OSError("serial port closed")
        return data

    def write(self, data):
        while data:
            try:
                n = os.write(self.fd, data)
            except BlockingIOError:
                select.select([], [self.fd], [], 0.1)
                continue
            data = data[n:]

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


EUROC_IMU_HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                    "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")


class ImuRecorder(threading.Thread):
    """Reads a Cube Orange over MAVLink, keeps IMU statistics for the PC app, and (while recording) writes
    <dataset>/mav0/imu0 (IMU1 = RAW_IMU), imu1 (IMU2), imu2 (IMU3) in EuRoC format plus mav0/cube/{attitude,timesync}.csv.

    Timestamps in data.csv are this computer's CLOCK_MONOTONIC (the same clock as the camera frame times); the Cube's own
    time stamps and the raw arrival times are in data_fc.csv, and timesync.csv holds TIMESYNC round trips (diagnostics).

    Toward the vehicle it only sends MAV_CMD_SET_MESSAGE_INTERVAL (telemetry rates on this link; reset at Cube reboot;
    asked back to default on shutdown) and TIMESYNC. Never arms, never writes parameters; rate requests are skipped
    while the vehicle is armed."""

    def __init__(self, port_arg, rate_hz, record, log=print):
        threading.Thread.__init__(self, name="imu", daemon=True)
        self.port_arg = port_arg
        self.rate_hz = float(rate_hz)
        self.record = bool(record)
        self.log = log
        self.enabled = str(port_arg).lower() not in ("none", "off", "")
        self.running = True
        self.ser = None
        self.parser = MavParser()
        self.seq = 0
        self.port = None
        self.state = "searching" if self.enabled else "disabled"
        self.sysid, self.compid = 1, 1
        self.armed = False
        self.autopilot = None
        self.arr = [collections.deque(maxlen=600) for _ in IMU_STREAMS]
        self.last_rx = 0.0
        self.touched = []
        self.lock = threading.Lock()
        self.rec = None
        self.rows = [0, 0, 0]
        self.t_first = None
        self.t_last = None
        self.offset_ns = None
        self.rtt_ns = None
        self.denied = 0
        self.rate_ladder = (100.0, 50.0, 40.0, 25.0)      # tried downwards when the Cube refuses a rate (loop rate too slow)
        self.last_step = 0.0
        self.outstanding = 0                              # rate requests sent that the Cube has not answered yet
        self.stale_acks = 0
        self.warned = set()
        self.last_request = 0.0
        self.last_sync = 0.0
        self.last_flush = 0.0
        self.sync_sent = collections.deque(maxlen=16)
        self.offs = collections.deque(maxlen=15)      # recent TIMESYNC clock offsets (Cube minus Jetson), ns (diagnostics)
        self.dwin = collections.deque(maxlen=1500)    # recent (Cube time - arrival time) of IMU1, ns
        self.dfilt = None                             # smoothed lowest-latency estimate of that difference
        self.last_t = [0, 0, 0]

    # -------------------------------------------------------------------------------------------- connection
    def _candidates(self):
        if str(self.port_arg).lower() != "auto":
            return [self.port_arg]
        c = []
        for pat in ("/dev/serial/by-id/*CubePilot*if00*", "/dev/serial/by-id/*Cube*if00*", "/dev/serial/by-id/*ArduPilot*if00*",
                    "/dev/ttyACM*"):
            for p in sorted(glob.glob(pat)):
                if p not in c:
                    c.append(p)
        return c

    def _warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.log(text)

    def _connect(self):
        for dev in self._candidates():
            try:
                ser = SerialPort(dev)
            except PermissionError:
                self.state = "no permission"
                self._warn_once("perm", "cannot open %s: permission denied - add this user to the 'dialout' group "
                                        "(sudo usermod -aG dialout $USER, then log in again)" % dev)
                continue
            except OSError:
                continue
            parser = MavParser()
            t0 = time.time()
            found = None
            try:
                while time.time() - t0 < 3.0 and found is None:
                    for msgid, sysid, compid, payload in parser.feed(ser.read(0.2)):
                        if msgid == 0:
                            hb = mav_decode(0, payload)
                            if hb["autopilot"] != 8:           # 8 = MAV_AUTOPILOT_INVALID (peripherals)
                                found = (sysid, compid, hb)
                                break
            except OSError:
                found = None
            if found is None:
                ser.close()
                continue
            self.ser, self.parser, self.port = ser, parser, dev
            self.sysid, self.compid = found[0], found[1]
            self.armed = bool(found[2]["base_mode"] & 128)
            self.autopilot = found[2]["autopilot"]
            self.last_rx = time.time()
            self.state = "connected"
            self.warned.discard("perm")
            self.log("IMU: connected to %s (system %d%s)" % (dev, self.sysid, ", ARMED" if self.armed else ""))
            self._request_rates()
            return True
        if self.state != "no permission":
            self.state = "searching"
        return False

    def _close(self):
        if self.ser is not None:
            self.ser.close()
        self.ser = None
        self.state = "searching"

    # -------------------------------------------------------------------------------------------- sending
    def _send(self, msgid, payload):
        self.seq = (self.seq + 1) & 0xFF
        self.ser.write(mav_pack2(msgid, payload, self.seq))

    def _set_interval(self, msgid, hz):
        interval = 0 if hz == 0 else int(round(1e6 / hz))
        payload = struct.pack("<fffffffHBBB", float(msgid), float(interval), 0, 0, 0, 0, 0,
                              MAV_CMD_SET_MESSAGE_INTERVAL, self.sysid, self.compid, 0)
        self._send(76, payload)

    def _request_rates(self):
        if self.armed:
            self._warn_once("armed", "IMU: vehicle is ARMED - not changing telemetry rates")
            return
        self.last_request = time.time()
        self.stale_acks = self.outstanding            # replies still owed for earlier requests
        for _, _, mid in IMU_STREAMS:
            self._set_interval(mid, self.rate_hz)
            if mid not in self.touched:
                self.touched.append(mid)
        self._set_interval(ATTITUDE_ID, 20.0)
        if ATTITUDE_ID not in self.touched:
            self.touched.append(ATTITUDE_ID)
        self.outstanding += len(IMU_STREAMS) + 1

    def _timesync(self):
        ts1 = time.monotonic_ns()
        self.sync_sent.append(ts1)
        self._send(111, struct.pack("<qq", 0, ts1))

    # -------------------------------------------------------------------------------------------- receiving
    def _stamp(self, idx, fc_ns, arrival_ns):
        """Sample time on this computer's CLOCK_MONOTONIC.

        Every message obeys  arrival = Cube_send_time - clock_offset + latency  with latency >= 0. The messages with the
        smallest latency therefore give the clock offset: D = max(Cube time - arrival) over a recent window (RAW_IMU, which has
        microsecond stamps; the same D serves IMU2/3). Sample time = Cube time - D, so USB / main-loop batching jitter is
        removed and a sample can never be placed after its arrival. (The TIMESYNC round trip is NOT used for this: the
        Cube answers only on its next 50 Hz loop tick, so the round trip is asymmetric and the NTP-style midpoint is biased
        by ~10 ms.) The constant minimum latency (~1 ms USB) stays inside D; VIO calibrates a remaining offset anyway.
        Falls back to the arrival time until 20 samples exist, and when the two disagree by more than 0.3 s (Cube reboot)."""
        if idx == 0:
            self.dwin.append(fc_ns - arrival_ns)
            if len(self.dwin) >= 20:
                best = max(self.dwin)
                self.dfilt = best if self.dfilt is None else self.dfilt + 0.05 * (best - self.dfilt)
        t = arrival_ns
        if self.dfilt is not None:
            cand = fc_ns - int(self.dfilt)
            if cand > arrival_ns:
                cand = arrival_ns
            if arrival_ns - cand < 300000000:
                t = cand
        if t <= self.last_t[idx]:
            t = self.last_t[idx] + 1000
        self.last_t[idx] = t
        return t

    def _handle(self, msgid, sysid, compid, payload, t_ns):
        if msgid == 0:
            hb = mav_decode(0, payload)
            if hb["autopilot"] != 8 and sysid == self.sysid:
                self.armed = bool(hb["base_mode"] & 128)
        elif msgid in (27, 116, 129):
            m = mav_decode(msgid, payload)
            idx = 0 if msgid == 27 else (1 if msgid == 116 else 2)
            self.arr[idx].append(t_ns * 1e-9)
            fc_us = m["time_usec"] if msgid == 27 else m["time_boot_ms"] * 1000
            arrival_ns = t_ns
            t_ns = self._stamp(idx, fc_us * 1000, arrival_ns)
            row = (m["xgyro"] * 1e-3, m["ygyro"] * 1e-3, m["zgyro"] * 1e-3,
                   m["xacc"] * 1e-3 * G_MSS, m["yacc"] * 1e-3 * G_MSS, m["zacc"] * 1e-3 * G_MSS)
            with self.lock:
                rec = self.rec
                if rec is not None and self.record:
                    rec["imu"][idx].write("%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n" % ((t_ns,) + row))
                    rec["fc"][idx].write("%d,%d,%d\n" % (t_ns, fc_us, arrival_ns))
                    self.rows[idx] += 1
                    if self.t_first is None:
                        self.t_first = t_ns
                    self.t_last = t_ns
        elif msgid == ATTITUDE_ID:
            m = mav_decode(msgid, payload)
            with self.lock:
                if self.rec is not None and self.record:
                    self.rec["att"].write("%d,%d,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n" % (
                        t_ns, m["time_boot_ms"], m["roll"], m["pitch"], m["yaw"], m["rollspeed"], m["pitchspeed"], m["yawspeed"]))
        elif msgid == 77:
            m = mav_decode(msgid, payload)
            if m["command"] == MAV_CMD_SET_MESSAGE_INTERVAL:
                # every request is answered; replies still owed for an EARLIER round (sent before the last step-down) must not
                # trigger another step. The time limit keeps a lost reply from blocking real refusals for ever.
                self.outstanding = max(0, self.outstanding - 1)
                stale = self.stale_acks > 0 and time.time() - self.last_step < 1.0
                if stale:
                    self.stale_acks -= 1
                if m["result"] != 0:
                    self.denied += 1
                    self._warn_once("denied%d" % m["result"], "IMU: Cube answered %s to a rate request (its main loop "
                                    "may be too slow for %g Hz)" % (ACK_NAMES.get(m["result"], m["result"]), self.rate_hz))
                    if not stale:                                    # refused in the current round: try the next lower rate
                        lower = [r for r in self.rate_ladder if r < self.rate_hz]
                        if lower:
                            self.last_step = time.time()
                            self.log("IMU: Cube refused %g Hz - asking for %g Hz instead" % (self.rate_hz, lower[0]))
                            self.rate_hz = lower[0]
                            self._request_rates()
        elif msgid == 111:
            m = mav_decode(msgid, payload)
            if m["tc1"] != 0 and m["ts1"] in self.sync_sent:
                rtt = t_ns - m["ts1"]
                if 0 < rtt < 500000000:
                    self.rtt_ns = rtt
                    self.offset_ns = m["tc1"] - (m["ts1"] + t_ns) // 2          # Cube time minus this computer's time
                    self.offs.append(self.offset_ns)
                    with self.lock:
                        if self.rec is not None and self.record:
                            self.rec["sync"].write("%d,%d,%d,%d,%d\n" % (m["ts1"], t_ns, m["tc1"], rtt, self.offset_ns))

    def run(self):
        if not self.enabled:
            return
        while self.running:
            if self.ser is None:
                if not self._connect():
                    time.sleep(2.0)
                continue
            try:
                data = self.ser.read(0.05)
                t_ns = time.monotonic_ns()
                if data:
                    self.last_rx = time.time()
                    for msgid, sysid, compid, payload in self.parser.feed(data):
                        self._handle(msgid, sysid, compid, payload, t_ns)
                now = time.time()
                if now - self.last_rx > 3.0:
                    self.log("IMU: no data for 3 s - reconnecting")
                    self._close()
                    continue
                if now - self.last_sync >= 1.0:
                    self.last_sync = now
                    self._timesync()
                if now - self.last_request >= 20.0 and not self.armed:       # a rebooted Cube forgets the rates
                    if self.snapshot()["hz"] < 0.7 * self.rate_hz:
                        self._request_rates()
                    else:
                        self.last_request = now
                if now - self.last_flush >= 1.0:
                    self.last_flush = now
                    with self.lock:
                        if self.rec is not None:
                            for grp in ("imu", "fc"):
                                for f in self.rec[grp]:
                                    f.flush()
                            for k in ("att", "sync"):
                                self.rec[k].flush()
            except OSError as e:
                self.log("IMU link lost: %s" % e)
                self._close()
                time.sleep(1.0)

    # -------------------------------------------------------------------------------------------- recording
    def is_connected(self):
        return self.ser is not None and time.time() - self.last_rx < 3.0

    def start_recording(self, root):
        """Open the IMU files of a new dataset (called when a recording starts)."""
        if not self.is_connected():
            return False
        base = os.path.join(root, "mav0")
        rec = {"imu": [], "fc": []}
        for k, (name, folder, _mid) in enumerate(IMU_STREAMS):
            d = os.path.join(base, folder)
            os.makedirs(d, exist_ok=True)
            f = open(os.path.join(d, "data.csv"), "w")
            f.write(EUROC_IMU_HEADER)
            g = open(os.path.join(d, "data_fc.csv"), "w")
            g.write("#timestamp [ns] (as in data.csv),cube_time_us,arrival_ns (raw, when the message was read)\n")
            rec["imu"].append(f)
            rec["fc"].append(g)
            with open(os.path.join(d, "sensor.yaml"), "w") as y:
                y.write("# ArduPilot %s over MAVLink (Cube Orange)\n" % name)
                y.write("sensor_type: imu\nrate_hz: %g\n" % self.rate_hz)
                y.write("comment: Cube IMU %d. Latest sample at the time the message was sent (not every raw sample), "
                        "16-bit integers (1 mg, 1 mrad/s). Timestamp = the Cube's own sample time moved onto the Jetson CLOCK_MONOTONIC using the lowest-latency "
                        "messages (raw arrival times and the Cube time are in data_fc.csv).\n" % (k + 1))
                y.write("frame: FRD (x forward, y right, z down); accelerometer = specific force (about -9.81 on z at rest)\n")
        cdir = os.path.join(base, "cube")
        os.makedirs(cdir, exist_ok=True)
        rec["att"] = open(os.path.join(cdir, "attitude.csv"), "w")
        rec["att"].write("#timestamp [ns],cube_time_ms,roll_rad,pitch_rad,yaw_rad,rollspeed,pitchspeed,yawspeed\n")
        rec["sync"] = open(os.path.join(cdir, "timesync.csv"), "w")
        rec["sync"].write("#jetson_send_ns,jetson_recv_ns,cube_ns,rtt_ns,offset_cube_minus_jetson_ns\n")
        with self.lock:
            self.rows = [0, 0, 0]
            self.t_first = self.t_last = None
            self.rec = rec
        return True

    def stop_recording(self):
        """Close the IMU files; returns a summary dict (or None if nothing was being written)."""
        with self.lock:
            rec, self.rec = self.rec, None
            rows = list(self.rows)
            t0, t1 = self.t_first, self.t_last
        if rec is None:
            return None
        for grp in ("imu", "fc"):
            for f in rec[grp]:
                f.close()
        rec["att"].close()
        rec["sync"].close()
        dur = (t1 - t0) / 1e9 if t0 is not None and t1 is not None and t1 > t0 else 0.0
        return {"port": self.port, "rate_requested_hz": self.rate_hz, "rows": rows, "duration_s": round(dur, 3),
                "measured_hz": round((rows[0] - 1) / dur, 2) if dur > 0 and rows[0] > 1 else 0.0,
                "clock_offset_used_ms": None if self.dfilt is None else round(self.dfilt / 1e6, 3),
                "clock_offset_timesync_ms": None if self.offset_ns is None else round(self.offset_ns / 1e6, 3),
                "timesync_rtt_ms": None if self.rtt_ns is None else round(self.rtt_ns / 1e6, 3)}

    # -------------------------------------------------------------------------------------------- reporting
    def snapshot(self):
        now = time.monotonic()
        a = [t for t in list(self.arr[0]) if now - t <= 2.0]
        hz, gap = 0.0, 0.0
        if len(a) >= 3 and now - a[-1] < 1.0:
            hz = (len(a) - 1) / (a[-1] - a[0])
            gap = max(b - c for b, c in zip(a[1:], a[:-1])) * 1000.0
        connected = self.is_connected()
        return {"enabled": self.enabled, "state": self.state if not connected else "connected", "port": self.port,
                "armed": self.armed, "hz": round(hz, 1), "gap_ms": round(gap), "rows": self.rows[0], "record": self.record,
                "requested_hz": self.rate_hz, "denied": self.denied,
                "offset_ms": None if self.offset_ns is None else round(self.offset_ns / 1e6, 1),
                "rtt_ms": None if self.rtt_ns is None else round(self.rtt_ns / 1e6, 1)}

    def shutdown(self):
        """Leave the Cube as we found it: ask for the default rates back, then close."""
        self.running = False
        try:
            if self.ser is not None and self.touched and not self.armed:
                for mid in self.touched:
                    self._set_interval(mid, 0)
                time.sleep(0.4)
        except OSError:
            pass
        self.stop_recording()
        self._close()


# ----------------------------------------------------------------------------
# Application
# ----------------------------------------------------------------------------
class App(object):
    def __init__(self, args):
        self.args = args
        self.cams = [Camera(0, args.role0, args.left, args)]
        if args.cameras == 2:
            self.cams.append(Camera(1, args.role1, args.right, args))
        self.swapping = False
        self.last_dataset = None
        self.writer = None
        self.rec_lock = threading.Lock()
        if len(self.cams) == 2:
            self.pairer = Pairer(self.cams[0], self.cams[1], args.fps, self.on_pair, args.max_skew_ms)
        else:
            self.pairer = Feeder(self.cams[0], self.on_pair)
        self.previews = [Preview(c, args, self) for c in self.cams]
        self.imu = ImuRecorder(args.imu_port, args.imu_rate, args.imu_record, log=lambda t: print("\n[imu] " + t))
        self.running = True
        self.rec_started = 0.0
        self.disk_free_gb = 0.0
        self.last_msg = ""
        self.msg_time = 0.0
        self.exposure = None        # last exposure set remotely, units of 100 us
        self.gain = None
        for ctrl in args.set_ctrl:   # remember --set-ctrl values for the status reply
            k, _, v = ctrl.partition("=")
            if k == "exposure_absolute" and v.isdigit():
                self.exposure = int(v)
            elif k == "gain" and v.isdigit():
                self.gain = int(v)

    # -- pairing callback ---------------------------------------------------
    def on_pair(self, l, r, skew):
        w = self.writer
        if w is not None and w.want(l.t_ns):
            w.submit(l, r, skew)

    # -- recording control ----------------------------------------------------
    def start_recording(self):
        with self.rec_lock:
            if self.writer is not None:
                return
            free = shutil.disk_usage(self.args.out if os.path.isdir(self.args.out) else
                                     os.path.dirname(os.path.abspath(self.args.out)) or ".").free / 1e9
            if free < self.args.min_free_gb:
                print("REFUSING to record: only %.1f GB free (< --min-free-gb %.1f)" % (free, self.args.min_free_gb))
                return
            root = next_dataset_dir(self.args.out, self.args.name)
            self.writer = DatasetWriter(root, self.args, self.cams)
            if self.imu.record:
                if self.imu.start_recording(root):
                    self.writer.imu_info = {"recording": True, "port": self.imu.port, "layout": "mav0/imu0 (Cube IMU1), imu1, imu2, cube/"}
                else:
                    print("\n[imu] Cube not connected - recording WITHOUT the IMU (state: %s)" % self.imu.snapshot()["state"])
            self.rec_started = time.time()
            print("\n>>> RECORDING -> %s" % root)

    def stop_recording(self):
        with self.rec_lock:
            w, self.writer = self.writer, None
        if w is None:
            return
        info = self.imu.stop_recording()
        if info is not None:
            w.imu_info = info
        n = w.finish()
        self.last_dataset = w.root
        mb = w.bytes / 1e6
        print("\n>>> STOPPED: %d pairs, %.0f MB, %d write drops, %d errors -> %s"
              % (n, mb, w.write_drops, w.errors, w.root))

    def toggle_recording(self):
        if self.writer is None:
            self.start_recording()
        else:
            self.stop_recording()

    # -- HUD bits that need app-wide state ------------------------------------------
    def draw_status(self, img, cam):
        h = img.shape[0]
        w = self.writer
        mean_skew, max_skew = self.pairer.skew_stats_ms()
        if w is not None:
            el = int(time.time() - self.rec_started)
            cv2.circle(img, (img.shape[1] - 20, 20), 8, (0, 0, 255), -1)
            put_text(img, "REC %02d:%02d  %d pairs" % (el // 60, el % 60, w.written),
                     (img.shape[1] - 200, 46), (0, 0, 255))
        else:
            put_text(img, "not recording (press r)", (img.shape[1] - 190, 46), (200, 200, 200), 0.45)
        put_text(img, "skew %.1f/%.1f ms  drops q%d w%d  free %.0f GB"
                 % (mean_skew, max_skew, cam.q_drops, w.write_drops if w else 0, self.disk_free_gb),
                 (10, h - 12), scale=0.45)
        if self.exposure:
            put_text(img, "exp %.1f ms%s" % (self.exposure / 10.0, "  gain %d" % self.gain if self.gain else ""),
                     (10, 100), (200, 200, 255), 0.45)
        if time.time() - self.msg_time < 4.0 and self.last_msg:
            put_text(img, self.last_msg, (10, 122), (0, 255, 255), 0.5)

    # -- supervision ---------------------------------------------------------------------
    def watchdog(self):
        last_print = 0.0
        while self.running:
            time.sleep(0.5)
            now = time.time()
            try:
                self.disk_free_gb = shutil.disk_usage(
                    self.args.out if os.path.isdir(self.args.out) else ".").free / 1e9
            except Exception:
                pass
            if self.writer is not None and self.disk_free_gb < self.args.min_free_gb:
                print("\nDisk nearly full (%.1f GB): stopping recording" % self.disk_free_gb)
                self.stop_recording()
            if self.args.duration and self.writer is not None and now - self.rec_started >= self.args.duration:
                self.stop_recording()
                if self.args.autostart:
                    self.running = False
            for c in self.cams:
                if self.swapping:
                    break
                msg = c.poll_bus()
                grace = 5.0 if now - c.started_wall < 5.0 else 2.0
                stalled = c.pipe is not None and now - max(c.last_frame_wall, c.started_wall) > grace
                if msg or stalled:
                    print("\n[%s] %s -> restarting" % (c.name, msg or "no frames for %.0f s" % grace))
                    try:
                        c.stop()
                        time.sleep(1.0)
                        c.restarts += 1
                        c.start()
                    except Exception as e:
                        print("[%s] restart failed: %s" % (c.name, e))
            if now - last_print >= 2.0:
                last_print = now
                m, mx = self.pairer.skew_stats_ms()
                w = self.writer
                print("\r[%s | frames %d | skew %.1f/%.1f ms | unpaired L%d R%d | %s]   "
                      % (" | ".join("%s %.1ffps" % (c.name, c.fps()) for c in self.cams),
                         self.pairer.pairs, m, mx, self.pairer.unpaired[0], self.pairer.unpaired[1],
                         ("REC %d" % w.written) if w else "idle"), end="")
                sys.stdout.flush()

    # -- remote control (UDP, from stereo_ctl.py on the PC) --------------------------------
    def say(self, text):
        self.last_msg, self.msg_time = text, time.time()
        print("\n" + text)

    def set_v4l2(self, name, value):
        """Apply a v4l2 control to both cameras (runtime only, not persistent)."""
        if self.args.sim:
            return
        for c in self.cams:
            try:
                subprocess.call(["v4l2-ctl", "-d", c.device, "-c", "%s=%d" % (name, value)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError as e:
                print("v4l2-ctl failed: %s" % e)

    def swap_cameras(self):
        """Exchange which physical camera is cam0 / cam1 (roles stay), e.g. when the boresight lens
        turns out to be on the other /dev/video node."""
        self.swapping = True
        try:
            self.say("remote: swapping cameras ...")
            for c in self.cams:
                c.stop()
            a, b = self.cams
            a.device, b.device = b.device, a.device
            for c in self.cams:
                c.queue.clear()
                c.latest = None
                c.arrivals.clear()
                c.last_frame_wall = 0.0
                c.start()
            self.say("remote: cam0=%s  cam1=%s" % (a.device, b.device))
        finally:
            self.swapping = False

    def status(self):
        w = self.writer
        return {"recording": w is not None, "pairs": w.written if w else 0, "api": API_VERSION,
                "cameras": len(self.cams), "mode": "mono" if len(self.cams) == 1 else "stereo",
                "record_fps": self.args.record_fps, "imu": self.imu.snapshot(),
                "preview": [self.previews[0].pw, self.previews[0].ph, self.args.preview_fps],
                "dataset": w.root if w else None,
                "elapsed_s": int(time.time() - self.rec_started) if w else 0,
                "fps": [round(c.fps(), 1) for c in self.cams],
                "focus": [round(c.metrics[0]) for c in self.cams],
                "mean": [round(c.metrics[1]) for c in self.cams],
                "clip_pct": [round(c.metrics[2], 1) for c in self.cams],
                "focus_peak": [round(c.metrics[3]) for c in self.cams],
                "focus_ratio": [round(c.metrics[4], 3) for c in self.cams],
                "roles": [c.role for c in self.cams],
                "devices": [c.device for c in self.cams],
                "last_dataset": self.last_dataset,
                "peak_window_s": self.args.peak_window,
                "exposure_100us": self.exposure, "gain": self.gain,
                "free_gb": round(self.disk_free_gb, 1),
                "write_drops": w.write_drops if w else 0}

    def handle_command(self, line):
        parts = line.split()
        cmd = parts[0].lower() if parts else ""
        try:
            if cmd in ("rec", "start"):
                self.start_recording()
                self.say("remote: recording %s" % ("started" if self.writer else "NOT started (disk?)"))
            elif cmd == "stop":
                self.stop_recording()
                self.say("remote: recording stopped")
            elif cmd == "toggle":
                self.toggle_recording()
            elif cmd == "focus":
                for p in self.previews:
                    p.scores.clear()
                self.say("remote: focus peak reset")
            elif cmd in ("exp", "exposure") and len(parts) == 2:
                v = max(1, min(10000, int(parts[1])))      # units of 100 us
                self.set_v4l2("exposure_auto", 1)          # 1 = manual
                self.set_v4l2("exposure_absolute", v)
                self.exposure = v
                self.say("remote: exposure %.1f ms" % (v / 10.0))
            elif cmd == "gain" and len(parts) == 2:
                v = max(1, min(40, int(parts[1])))
                self.set_v4l2("gain", v)
                self.gain = v
                self.say("remote: gain %d" % v)
            elif cmd == "rate" and len(parts) == 2:
                if self.writer is not None:
                    return dict(self.status(), error="stop recording before changing the save rate")
                v = max(1.0, min(float(self.args.fps), float(parts[1])))
                self.args.record_fps = v
                self.say("remote: saving %.0f frames/s" % v)
            elif cmd == "imurec" and len(parts) == 2:
                if self.writer is not None:
                    return dict(self.status(), error="stop recording before changing the IMU setting")
                self.imu.record = parts[1] not in ("0", "off", "false")
                self.say("remote: IMU recording %s" % ("on" if self.imu.record else "off"))
            elif cmd == "swap":
                if len(self.cams) < 2:
                    return dict(self.status(), error="swap needs the two-camera mode")
                if self.writer is not None:
                    return dict(self.status(), error="stop recording before swapping the cameras")
                self.swap_cameras()
            elif cmd == "quit":
                self.running = False
            elif cmd != "status":
                return dict(self.status(), error="unknown command: %s" % line)
        except Exception as e:
            return dict(self.status(), error=str(e))
        return self.status()

    def control_server(self):
        port = self.args.control_port
        allowed = self.args.dest if self.args.dest not in ("", "0.0.0.0") else None
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.5)
        print("Remote control on udp %d (accepts commands from %s only)" % (port, allowed or "anyone"))
        while self.running:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if allowed and addr[0] != allowed and not addr[0].startswith("127."):
                continue
            reply = self.handle_command(data.decode("utf-8", "ignore").strip())
            try:
                sock.sendto(json.dumps(reply).encode(), addr)
            except OSError:
                pass
        sock.close()

    def keyboard(self):
        if not sys.stdin.isatty():
            return
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running:
                if select.select([sys.stdin], [], [], 0.2)[0]:
                    k = sys.stdin.read(1).lower()
                    if k == "r":
                        self.toggle_recording()
                    elif k == "f":
                        for p in self.previews:
                            p.scores.clear()
                        print("\nfocus peak reset")
                    elif k == "q":
                        self.running = False
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        a = self.args
        for c in self.cams:
            c.start()
        self.imu.start()
        self.pairer.start()
        for p in self.previews:
            p.start()
        threading.Thread(target=self.watchdog, name="watchdog", daemon=True).start()
        if a.control_port:
            threading.Thread(target=self.control_server, name="control", daemon=True).start()
        if not a.no_stream:
            print("Streaming preview to %s: %s" % (a.dest, ", ".join(
                "cam%d udp %d" % (c.index, a.left_port if c.index == 0 else a.right_port) for c in self.cams)))
            print("  PC viewer:  view_streams.bat   (or: gst-launch-1.0 udpsrc port=%d caps=\"application/x-rtp,"
                  "encoding-name=H264,payload=96\" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                  "autovideosink sync=false)" % a.left_port)
        print("Keys: r = start/stop recording, f = reset focus peak, q = quit\n")
        if a.autostart:
            time.sleep(1.5)
            self.start_recording()
        kb = threading.Thread(target=self.keyboard, name="keys", daemon=True)
        kb.start()
        try:
            while self.running:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        self.shutdown()

    def shutdown(self):
        self.running = False
        print("\nshutting down ...")
        self.stop_recording()
        self.imu.shutdown()
        self.pairer.running = False
        for p in self.previews:
            p.stop()
        for c in self.cams:
            c.stop()


# ----------------------------------------------------------------------------
# Environment check
# ----------------------------------------------------------------------------
def run_check(args):
    print("python      :", sys.version.split()[0])
    print("GStreamer   :", Gst.version_string())
    print("OpenCV      :", cv2.__version__, "| numpy", np.__version__)
    for el in ("v4l2src", "nvvidconv", "nvv4l2h264enc", "x264enc", "appsink", "appsrc", "rtph264pay"):
        print("  element %-14s %s" % (el, "ok" if have_element(el) else "MISSING"))
    for dev in ([args.left] + ([args.right] if args.cameras == 2 else [])):
        exists = os.path.exists(dev)
        print("device %-16s %s%s" % (dev, "present" if exists else "MISSING",
                                      " -> " + os.path.realpath(dev) if exists else ""))
        if exists and shutil.which("v4l2-ctl"):
            out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True).stdout
            keep = [l.strip() for l in out.splitlines() if "UYVY" in l or "YUYV" in l or "MJPG" in l or
                    ("%dx%d" % (args.width, args.height)) in l]
            for l in keep[:8]:
                print("    ", l)
    os.makedirs(args.out, exist_ok=True)
    print("output dir  : %s  (%.1f GB free)" % (args.out, shutil.disk_usage(args.out).free / 1e9))
    print("RTC         :", read_rtc_epoch())


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--left", default="/dev/boresight", help="cam0 device (default %(default)s)")
    p.add_argument("--right", default="/dev/depression", help="cam1 device (default %(default)s)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=60, help="camera frame rate (default %(default)s)")
    p.add_argument("--format", default="UYVY", help="UYVY (default), YUYV or MJPG")
    p.add_argument("--dest", default="192.168.0.5", help="PC IP for the preview stream (default %(default)s)")
    p.add_argument("--left-port", type=int, default=5000)
    p.add_argument("--right-port", type=int, default=5002)
    p.add_argument("--no-stream", action="store_true", help="do not send the preview")
    p.add_argument("--preview-fps", type=int, default=15)
    p.add_argument("--preview-width", type=int, default=640)
    p.add_argument("--stream-kbps", type=int, default=3000, help="H.264 bitrate per camera")
    p.add_argument("--peak-window", type=int, default=20, help="seconds the focus peak is held")
    p.add_argument("--out", default=os.path.expanduser("~/recordings"))
    p.add_argument("--name", help="dataset folder name (default stereo_0001, 0002, ...)")
    p.add_argument("--record-fps", type=float, default=30.0, help="pairs saved per second (default %(default)s)")
    p.add_argument("--png-level", type=int, default=1, choices=range(0, 10),
                   help="PNG compression 0-9; 1 is fast, 6+ saves space but costs CPU (default %(default)s)")
    p.add_argument("--writers", type=int, default=3, help="PNG writer threads")
    p.add_argument("--max-skew-ms", type=float, default=None,
                   help="max left/right time difference to pair (default half a frame period)")
    p.add_argument("--min-free-gb", type=float, default=1.0)
    p.add_argument("--autostart", action="store_true", help="start recording immediately")
    p.add_argument("--duration", type=float, default=0, help="stop a recording after N seconds "
                   "(with --autostart the program exits afterwards)")
    p.add_argument("--set-ctrl", action="append", default=[], metavar="NAME=VALUE",
                   help="v4l2 control applied to both cameras before start, e.g. exposure_auto=1 (repeatable)")
    p.add_argument("--control-port", type=int, default=5010,
                   help="UDP port for remote start/stop/exposure from stereo_ctl.py on the PC; 0 disables "
                        "(default %(default)s). Only commands from --dest are accepted.")
    p.add_argument("--cameras", type=int, choices=(1, 2), default=2,
                   help="1 = record only the boresight camera (mono), 2 = both (default %(default)s)")
    p.add_argument("--hud", action="store_true",
                   help="draw the focus meter / record state into the preview (default: clean picture)")
    p.add_argument("--role0", default="boresight 6mm", help="label for cam0 (default %(default)s)")
    p.add_argument("--role1", default="depression 8mm", help="label for cam1 (default %(default)s)")
    p.add_argument("--imu-port", default="auto",
                   help="Cube Orange serial port for the IMU: auto (default), a device like /dev/ttyACM0, or none")
    p.add_argument("--imu-rate", type=float, default=100.0,
                   help="IMU message rate to ask the Cube for, Hz. The Cube accepts up to about half its main loop rate "
                        "(SCHED_LOOP_RATE 200 -> 100 Hz, 50 -> 40 Hz); if it refuses, the request steps down 100/50/40/25 "
                        "(default %(default)s)")
    p.add_argument("--no-imu-record", dest="imu_record", action="store_false",
                   help="read the IMU (status only) but do not write it into recordings")
    p.add_argument("--note", default="", help="free text stored in meta.json")
    p.add_argument("--sim", action="store_true", help="synthetic cameras, for testing without hardware")
    p.add_argument("--check", action="store_true", help="print the environment and camera status, then exit")
    return p.parse_args(argv)


def main():
    args = parse_args()
    if args.check:
        run_check(args)
        return
    if not args.sim:
        devs = [args.left] + ([args.right] if args.cameras == 2 else [])
        for dev in devs:
            if not os.path.exists(dev):
                sys.exit("camera %s not found (run with --check; is setup_cameras.sh done?)" % dev)
        for ctrl in args.set_ctrl:
            for dev in devs:
                subprocess.call(["v4l2-ctl", "-d", dev, "-c", ctrl])
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    App(args).run()


if __name__ == "__main__":
    main()
