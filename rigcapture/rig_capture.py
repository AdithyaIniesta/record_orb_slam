#!/usr/bin/env python3
"""
Rig Capture  -  recording ground station for the Jetson camera rig.

Same look and feel as the CvTracker Ground Station (jvp-ui): dark/light/sunlight themes, connection bar on top,
control column on the left, big video on the right, log at the bottom.

  * live preview of the boresight camera (mono) or both cameras (boresight over depression)
  * START RECORDING / STOP RECORDING with the mouse (optional countdown so you can pick the rig up first)
  * exposure + gain sliders, recording / disk status, list of recordings on the Jetson with one-click copy to this PC
  * starts and updates the capture program on the Jetson by itself (ssh), so double-clicking the .exe is all it takes

Focus is judged by eye - there is deliberately no focus meter here.

Video: H.264/RTP from the Jetson (UDP 5000 / 5002) decoded by GStreamer (gst-launch-1.0) or ffmpeg, exactly like jvp-ui.
Control: UDP 5010 to stereo_capture.py on the Jetson.  Setup / copy: the Windows OpenSSH client (ssh, tar).
"""
import argparse
import json
import os
import posixpath
import queue
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk

APP_NAME = "Rig Capture"
APP_VERSION = "1.0"
REQUIRED_API = 5        # capture-program interface this app needs (stereo_capture.py API_VERSION)
NO_WINDOW = 0x08000000 if os.name == "nt" else 0

# ==========================================================================
# Themes - the same three palettes as jvp-ui
# ==========================================================================
THEMES = {
    "Light": {
        "BG": "#f0f0f0", "PANEL_BG": "#f0f0f0", "ROW_EVEN": "#f7f7f7", "ROW_ODD": "#ffffff",
        "HDR_BG": "#e0e0e0", "HDR_FG": "#333333", "RO_FG": "#1a1a6e", "EDIT_FG": "#cc5500",
        "EDIT_BG": "#fffbe6", "PARAM_FG": "#004400", "MODE_COLOR": "#003399", "SEC_FG": "#555555",
        "LOG_BG": "#ffffff", "LOG_FG": "#000000", "INPUT_BG": "#ffffff", "INPUT_FG": "#000000",
        "SEP": "#c8c8c8", "ARROW": "#333333", "ACCENT": "#1d6fd6",
    },
    "Dark": {
        "BG": "#1e1e22", "PANEL_BG": "#26262b", "ROW_EVEN": "#2a2a30", "ROW_ODD": "#232328",
        "HDR_BG": "#33333a", "HDR_FG": "#e8e8ea", "RO_FG": "#7fb8ff", "EDIT_FG": "#ffb066",
        "EDIT_BG": "#403520", "PARAM_FG": "#7ee38a", "MODE_COLOR": "#ffd24d", "SEC_FG": "#a0a0a8",
        "LOG_BG": "#161619", "LOG_FG": "#e2e2e5", "INPUT_BG": "#1a1a1e", "INPUT_FG": "#f0f0f2",
        "SEP": "#3a3a42", "ARROW": "#e8e8ea", "ACCENT": "#4c9dff",
    },
    "Sunlight": {
        "BG": "#0d0d0f", "PANEL_BG": "#141416", "ROW_EVEN": "#191919", "ROW_ODD": "#0f0f0f",
        "HDR_BG": "#1e1e20", "HDR_FG": "#ffe08a", "RO_FG": "#ffe08a", "EDIT_FG": "#ff9147",
        "EDIT_BG": "#2a1e10", "PARAM_FG": "#75e37f", "MODE_COLOR": "#ffcc33", "SEC_FG": "#c7c7c0",
        "LOG_BG": "#0a0a0c", "LOG_FG": "#ffe8a8", "INPUT_BG": "#141416", "INPUT_FG": "#fff2c8",
        "SEP": "#3a3320", "ARROW": "#ffe08a", "ACCENT": "#ffcc33",
    },
}
DEFAULT_THEME = "Dark"
REC_GREEN, REC_RED = "#2e9e4f", "#c0392b"

# design sizes at 1080p; everything scales with the screen height like jvp-ui
_FONT_BASE, _FONT_MONO, _FONT_VALUE = 13, 14, 16

VIDEO_ACTIVE_TIMEOUT = 5.0
STREAM_STALL_TIMEOUT_S = 6.0
STATUS_STALE_S = 1.5

MONO_PREVIEW = (960, 540, 20, 4000)      # width, height, fps, kbps - one big picture
STEREO_PREVIEW = (640, 360, 15, 3000)    # two pictures stacked


def app_dir():
    """Folder of the .exe (or of this script) - where the config file lives."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def resource(name):
    """File bundled into the exe (PyInstaller --add-data) or sitting next to the script."""
    for base in (getattr(sys, "_MEIPASS", None), app_dir(), os.path.join(app_dir(), "jetson")):
        if base and os.path.isfile(os.path.join(base, name)):
            return os.path.join(base, name)
    return None


# ==========================================================================
# VideoReceiver - H.264/RTP via GStreamer or ffmpeg (taken from jvp-ui, RGB instead of BGR)
# ==========================================================================
class VideoReceiver:
    def __init__(self, on_frame, on_status, on_log=None, label="Stream"):
        self.on_frame = on_frame
        self.on_status = on_status
        self.on_log = on_log or (lambda s: None)
        self.label = label
        self.port = 5000
        self.width = 960
        self.height = 540
        self.running = True
        self.restart = threading.Event()
        self.proc = None
        self._lock = threading.Lock()
        self._stderr_lines = []
        self._last_data_t = 0.0
        self.stall_timeout = STREAM_STALL_TIMEOUT_S
        self.paused = False
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._watchdog, daemon=True).start()

    def configure(self, port, width, height):
        self.port, self.width, self.height = int(port), int(width), int(height)
        self.paused = False
        self.restart.set()
        self._kill()

    def standby(self):
        self.paused = True
        self._last_data_t = 0.0
        self.restart.set()
        self._kill()
        self.on_status("[%s] standby" % self.label)

    @staticmethod
    def _find_gst():
        exe = shutil.which("gst-launch-1.0")
        if exe:
            return exe
        for c in (r"C:\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe",
                  r"C:\gstreamer\1.0\mingw_x86_64\bin\gst-launch-1.0.exe",
                  r"C:\Program Files\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe"):
            if os.path.exists(c):
                return c
        return None

    @staticmethod
    def _find_ffmpeg():
        for base in (getattr(sys, "_MEIPASS", None), os.path.dirname(getattr(sys, "executable", "")), app_dir()):
            if base and os.path.isfile(os.path.join(base, "ffmpeg.exe")):
                return os.path.join(base, "ffmpeg.exe")
        return shutil.which("ffmpeg")

    def _gst_cmd(self, exe):
        # reuse=false / buffer-size / rtpjitterbuffer: see jvp-ui - a second binder of the same UDP port silently steals
        # the stream on Windows, and a keyframe burst overflows the default 64 KB socket buffer.
        return [exe, "-q", "udpsrc", "port=%d" % self.port, "reuse=false", "buffer-size=8388608",
                "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96",
                "!", "rtpjitterbuffer", "latency=200", "do-lost=true",
                "!", "rtph264depay", "!", "h264parse", "!", "avdec_h264",
                "!", "videoconvert", "!", "videoscale",
                "!", "video/x-raw,format=RGB,width=%d,height=%d" % (self.width, self.height),
                "!", "fdsink", "fd=1"]

    def _ffmpeg_cmd(self, exe):
        sdp = ("v=0\no=- 0 0 IN IP4 0.0.0.0\ns=rigcapture\nc=IN IP4 0.0.0.0\nt=0 0\n"
               "m=video %d RTP/AVP 96\na=rtpmap:96 H264/90000\n" % self.port)
        path = os.path.join(tempfile.gettempdir(), "rigcapture_stream_%d.sdp" % self.port)
        with open(path, "w") as f:
            f.write(sdp)
        return [exe, "-loglevel", "error", "-protocol_whitelist", "file,udp,rtp",
                "-analyzeduration", "3000000", "-probesize", "3000000", "-fflags", "nobuffer", "-flags", "low_delay",
                "-i", path, "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (self.width, self.height), "-"]

    def _kill(self):
        with self._lock:
            proc, self.proc = self.proc, None
        if proc is None:
            return
        for _ in range(3):
            try:
                proc.kill()
                proc.wait(timeout=2)
                return
            except subprocess.TimeoutExpired:
                continue
            except Exception:
                return
        self.on_log("!! [%s] decoder for UDP :%d would not exit - it may still hold the port" % (self.label, self.port))

    def shutdown(self):
        self.running = False
        self._last_data_t = 0.0
        self.restart.set()
        self._kill()

    def _watchdog(self):
        while self.running:
            time.sleep(1.0)
            last = self._last_data_t
            if last <= 0.0 or time.monotonic() - last < self.stall_timeout:
                continue
            self._last_data_t = 0.0
            self.on_log("!! [%s] no video for %.0fs on UDP :%d - restarting receiver"
                        % (self.label, self.stall_timeout, self.port))
            self.restart.set()
            self._kill()

    def _loop(self):
        while self.running:
            self.restart.clear()
            if self.paused:
                time.sleep(0.2)
                continue
            gst, ff = self._find_gst(), self._find_ffmpeg()
            if gst:
                cmd, name = self._gst_cmd(gst), "GStreamer"
            elif ff:
                cmd, name = self._ffmpeg_cmd(ff), "ffmpeg"
            else:
                self.on_status("[%s] install GStreamer or put ffmpeg.exe next to the app" % self.label)
                time.sleep(3.0)
                continue
            self.on_status("[%s] %s: listening on UDP :%d ..." % (self.label, name, self.port))
            try:
                with self._lock:
                    if not self.running:
                        break
                    self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                                 creationflags=NO_WINDOW)
                self._last_data_t = time.monotonic()
                self._stderr_lines = []
                threading.Thread(target=self._drain_stderr, args=(self.proc,), daemon=True).start()
            except OSError as e:
                self.on_status("[%s] %s failed to start: %s" % (self.label, name, e))
                time.sleep(3.0)
                continue
            nbytes = self.width * self.height * 3
            got_any = False
            start_t = time.monotonic()
            proc = self.proc
            if proc is None:
                continue
            while self.running and not self.restart.is_set():
                buf = b""
                while len(buf) < nbytes:
                    try:
                        chunk = proc.stdout.read(nbytes - len(buf))
                    except (OSError, ValueError):
                        chunk = None
                    if not chunk:
                        buf = None
                        break
                    buf += chunk
                    self._last_data_t = time.monotonic()
                if buf is None:
                    break
                if not got_any:
                    got_any = True
                    self.on_status("[%s] %s: stream connected" % (self.label, name))
                self.on_frame(buf, self.width, self.height)
            self._kill()
            self._last_data_t = 0.0
            if self.running and not self.restart.is_set():
                if not got_any and time.monotonic() - start_t < 2.0:
                    tail = "\n".join(self._stderr_lines[-5:]).strip()
                    self.on_log("!! [%s] %s exited%s" % (self.label, name, (":\n" + tail) if tail else
                                " immediately - is UDP :%d already in use?" % self.port))
                self.on_status("[%s] %s: ended - reconnecting" % (self.label, name))
                time.sleep(1.0)
        self.on_status("[%s] stopped - port released" % self.label)

    def _drain_stderr(self, proc):
        try:
            for raw in iter(proc.stderr.readline, b""):
                line = raw.decode(errors="replace").rstrip()
                if line:
                    self._stderr_lines.append(line)
                    del self._stderr_lines[:-50]
        except (OSError, ValueError):
            pass


# ==========================================================================
# Control channel to stereo_capture.py on the Jetson (UDP)
# ==========================================================================
def udp_send(ip, port, cmd, timeout=0.6):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(cmd.encode(), (ip, port))
        data, _ = s.recvfrom(16384)
        return json.loads(data.decode())
    except (socket.timeout, OSError, ValueError):
        return None
    finally:
        s.close()


def local_ip_towards(ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((ip, 9))
        return s.getsockname()[0]
    except OSError:
        return "0.0.0.0"
    finally:
        s.close()


REMOTE_LIST_PY = r'''
import json, os, glob
out = []
for d in sorted(glob.glob(os.path.expanduser("~/recordings/*/"))):
    n = os.path.basename(d.rstrip("/"))
    m = {}
    try:
        m = json.load(open(os.path.join(d, "meta.json")))
    except Exception:
        pass
    size = 0
    for r, _, fs in os.walk(d):
        for f in fs:
            try:
                size += os.path.getsize(os.path.join(r, f))
            except OSError:
                pass
    out.append({"name": n, "complete": m.get("complete"), "frames": m.get("frames_written", m.get("pairs_written")),
                "dur": m.get("duration_s"), "mb": round(size / 1e6), "cams": m.get("cameras", 2)})
print(json.dumps(out))
'''


# ==========================================================================
# The application window
# ==========================================================================
class App:
    def __init__(self, args):
        self.args = args
        self.root = tk.Tk()
        self.root.title("%s %s" % (APP_NAME, APP_VERSION))
        scr_w, scr_h = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        win_w, win_h = min(1900, int(scr_w * 0.95)), min(1060, int(scr_h * 0.92))
        self.root.geometry("%dx%d+%d+%d" % (win_w, win_h, max(0, (scr_w - win_w) // 2), max(0, (scr_h - win_h) // 2)))
        self.root.minsize(min(1200, int(scr_w * 0.8)), min(700, int(scr_h * 0.8)))
        if not args.shot:
            try:
                self.root.state("zoomed")
            except tk.TclError:
                pass

        self.cfg_path = os.path.join(app_dir(), "rig_capture_config.json")
        self.cfg = self._load_cfg()
        if args.jetson:
            self.cfg["ip"] = args.jetson
        if args.user:
            self.cfg["user"] = args.user
        if args.theme:
            self.cfg["theme"] = args.theme
        if args.video_port:
            self.cfg["video_port"] = args.video_port
        if args.cmd_port:
            self.cfg["cmd_port"] = args.cmd_port

        self.status = None
        self.status_t = 0.0
        self.ssh_ok = None
        self.busy = False
        self.countdown_left = 0
        self._countdown_job = None
        self.frame_q = [queue.Queue(maxsize=2), queue.Queue(maxsize=2)]
        self.frame_t = [0.0, 0.0]
        self.fps_times = [[], []]
        self.photo = [None, None]
        self.img_id = [None, None]
        self.pv_size = [None, None]
        self.local_sets = set()
        self._themed = []             # (widget, {option: palette key})
        self._apply_job = None

        self._init_visual()
        self._build_ui()
        self.rx = [VideoReceiver(lambda b, w, h, i=i: self._frame_cb(i, b, w, h),
                                 lambda t, i=i: self.root.after(0, lambda: self.vstatus[i].set(t)),
                                 self.log, label="Boresight" if i == 0 else "Depression") for i in range(2)]
        self._apply_connection(first=True)
        self._apply_theme(self.theme_var.get())
        threading.Thread(target=self._poll_status, daemon=True).start()
        self._poll_frames()
        self._tick()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.log("%s %s - Jetson %s" % (APP_NAME, APP_VERSION, self.cfg["ip"]))
        if args.shot:
            self.root.after(int(args.shot_after * 1000), self._shot)
            if args.test_rec:       # self-test: press START at shot_after-9 s (runs the countdown), screenshot shows REC
                self.root.after(int((args.shot_after - 9) * 1000), self._on_start)
                self.root.after(int((args.shot_after - 6) * 1000), self._test_exposure)
        if self.cfg.get("autostart", True) and not args.no_autostart:
            self.root.after(1500, self._auto_connect)

    # -- config ------------------------------------------------------------------------------------
    def _load_cfg(self):
        d = {"ip": "192.168.0.40", "user": "orinnx1", "video_port": 5000, "cmd_port": 5010, "cameras": 1,
             "theme": DEFAULT_THEME, "exposure_ms": 4.0, "gain": 1, "delay_s": 3, "record_fps": 30, "imu": True, "autostart": True,
             "dest_dir": os.path.join(os.path.expanduser("~"), "Documents", "RigCapture", "recordings")}
        try:
            with open(self.cfg_path, "r", encoding="utf-8") as f:
                d.update(json.load(f) or {})
        except Exception:
            pass
        return d

    def _save_cfg(self):
        try:
            with open(self.cfg_path, "w", encoding="utf-8") as f:
                json.dump(self.cfg, f, indent=2)
        except OSError:
            pass

    # -- look: fonts and styles exactly in jvp-ui's manner --------------------------------------------
    def _init_visual(self):
        h = self.root.winfo_screenheight() or 1080
        self._scale = max(0.9, min(2.0, h / 1080.0))
        sz = lambda base: max(9, int(round(base * self._scale)))
        r = self.root
        self.F_MONO = tkfont.Font(root=r, family="Consolas", size=sz(_FONT_MONO))
        self.F_MONO_B = tkfont.Font(root=r, family="Consolas", size=sz(_FONT_MONO), weight="bold")
        self.F_MONO_V = tkfont.Font(root=r, family="Consolas", size=sz(_FONT_VALUE), weight="bold")
        self.F_UI = tkfont.Font(root=r, family="Segoe UI", size=sz(_FONT_BASE))
        self.F_UI_B = tkfont.Font(root=r, family="Segoe UI", size=sz(_FONT_BASE), weight="bold")
        self.F_BIG = tkfont.Font(root=r, family="Segoe UI", size=sz(_FONT_BASE + 4), weight="bold")
        try:
            tkfont.nametofont("TkDefaultFont").configure(size=sz(_FONT_BASE))
            tkfont.nametofont("TkTextFont").configure(size=sz(_FONT_BASE))
            tkfont.nametofont("TkFixedFont").configure(size=sz(_FONT_MONO))
        except Exception:
            pass
        self.theme_var = tk.StringVar(value=self.cfg.get("theme") if self.cfg.get("theme") in THEMES else DEFAULT_THEME)
        self.style = ttk.Style(self.root)
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

    def reg(self, widget, **opts):
        """Register a plain tk widget for re-colouring: reg(w, bg='PANEL_BG', fg='HDR_FG')."""
        self._themed.append((widget, opts))
        t = THEMES[self.theme_var.get()]
        try:
            widget.configure(**dict((k, t[v]) for k, v in opts.items()))
        except tk.TclError:
            pass
        return widget

    def _apply_theme(self, name):
        if name not in THEMES:
            return
        t = THEMES[name]
        self.cfg["theme"] = name
        self._save_cfg()
        self.root.configure(bg=t["BG"])
        st = self.style
        st.configure(".", background=t["PANEL_BG"], foreground=t["HDR_FG"], font=self.F_UI, fieldbackground=t["INPUT_BG"])
        st.configure("TFrame", background=t["PANEL_BG"])
        st.configure("TLabel", background=t["PANEL_BG"], foreground=t["HDR_FG"])
        st.configure("TLabelframe", background=t["PANEL_BG"], bordercolor=t["SEP"])
        st.configure("TLabelframe.Label", background=t["PANEL_BG"], foreground=t["HDR_FG"], font=self.F_UI_B)
        st.configure("TButton", padding=(6, 4), background=t["HDR_BG"], foreground=t["HDR_FG"], bordercolor=t["SEP"])
        st.map("TButton", background=[("active", t["ACCENT"]), ("disabled", t["ROW_ODD"])],
               foreground=[("disabled", t["SEC_FG"])])
        st.configure("TEntry", fieldbackground=t["INPUT_BG"], foreground=t["INPUT_FG"], insertcolor=t["INPUT_FG"])
        st.configure("TCombobox", fieldbackground=t["INPUT_BG"], foreground=t["INPUT_FG"], background=t["HDR_BG"],
                     arrowcolor=t["ARROW"], selectbackground=t["INPUT_BG"], selectforeground=t["INPUT_FG"])
        st.map("TCombobox", fieldbackground=[("readonly", t["INPUT_BG"])], foreground=[("readonly", t["INPUT_FG"])])
        st.configure("TCheckbutton", background=t["PANEL_BG"], foreground=t["HDR_FG"])
        st.map("TCheckbutton", background=[("active", t["PANEL_BG"])])
        st.configure("TScale", background=t["PANEL_BG"], troughcolor=t["HDR_BG"])
        st.configure("TSeparator", background=t["SEP"])
        st.configure("TPanedwindow", background=t["BG"])
        st.configure("TScrollbar", background=t["HDR_BG"], troughcolor=t["PANEL_BG"], arrowcolor=t["ARROW"])
        st.configure("Treeview", background=t["ROW_ODD"], fieldbackground=t["ROW_ODD"], foreground=t["HDR_FG"],
                     rowheight=int(26 * self._scale), font=self.F_UI)
        st.configure("Treeview.Heading", background=t["HDR_BG"], foreground=t["HDR_FG"], font=self.F_UI_B)
        st.map("Treeview", background=[("selected", t["ACCENT"])], foreground=[("selected", "#ffffff")])
        for w, opts in self._themed:
            try:
                w.configure(**dict((k, t[v]) for k, v in opts.items()))
            except tk.TclError:
                pass
        self._paint_rows()

    # -- UI -------------------------------------------------------------------------------------------------
    def _build_ui(self):
        root = self.root
        top = ttk.Frame(root, padding=(10, 8))
        top.pack(side=tk.TOP, fill=tk.X)

        def lbl(t):
            ttk.Label(top, text=t).pack(side=tk.LEFT, padx=(6, 2))

        lbl("Jetson IP")
        self.ip_var = tk.StringVar(value=self.cfg["ip"])
        ttk.Combobox(top, textvariable=self.ip_var, values=[self.cfg["ip"], "192.168.0.30", "192.168.0.40"],
                     width=14, state="normal").pack(side=tk.LEFT, padx=2)
        lbl("SSH user")
        self.user_var = tk.StringVar(value=self.cfg["user"])
        ttk.Entry(top, textvariable=self.user_var, width=10).pack(side=tk.LEFT, padx=1)
        lbl("Video")
        self.vport_var = tk.StringVar(value=str(self.cfg["video_port"]))
        ttk.Entry(top, textvariable=self.vport_var, width=5).pack(side=tk.LEFT, padx=1)
        lbl("Cmd")
        self.cport_var = tk.StringVar(value=str(self.cfg["cmd_port"]))
        ttk.Entry(top, textvariable=self.cport_var, width=5).pack(side=tk.LEFT, padx=1)
        lbl("Cameras")
        self.cams_var = tk.StringVar(value="Boresight only (mono)" if self.cfg["cameras"] == 1 else "Both cameras")
        ttk.Combobox(top, textvariable=self.cams_var, values=["Boresight only (mono)", "Both cameras"],
                     width=20, state="readonly").pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Apply", command=self._apply_connection).pack(side=tk.LEFT, padx=6)
        lbl("Theme")
        tc = ttk.Combobox(top, textvariable=self.theme_var, values=list(THEMES), width=9, state="readonly")
        tc.pack(side=tk.LEFT, padx=2)
        tc.bind("<<ComboboxSelected>>", lambda _e: self._apply_theme(self.theme_var.get()))
        self.vstatus = [tk.StringVar(value="Boresight: -"), tk.StringVar(value="Depression: -")]
        ttk.Label(top, textvariable=self.vstatus[0], foreground="#888888").pack(side=tk.LEFT, padx=(10, 4))
        self.vstatus_lbl2 = ttk.Label(top, textvariable=self.vstatus[1], foreground="#888888")
        self.vstatus_lbl2.pack(side=tk.LEFT, padx=4)
        ttk.Separator(root, orient="horizontal").pack(side=tk.TOP, fill=tk.X)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.body = body

        # LEFT: scrollable column
        left_outer = ttk.Frame(body)
        left_outer.rowconfigure(0, weight=1)
        left_outer.columnconfigure(0, weight=1)
        body.add(left_outer, weight=0)
        lc = tk.Canvas(left_outer, highlightthickness=0, width=int(560 * self._scale))
        self.reg(lc, bg="BG")
        sb = ttk.Scrollbar(left_outer, orient=tk.VERTICAL, command=lc.yview)
        lc.configure(yscrollcommand=sb.set)
        lc.grid(row=0, column=0, sticky="nsew")
        sb.grid(row=0, column=1, sticky="ns")
        left = ttk.Frame(lc)
        wid = lc.create_window((0, 0), window=left, anchor="nw")
        left.bind("<Configure>", lambda e: lc.configure(scrollregion=lc.bbox("all")))
        lc.bind("<Configure>", lambda e: lc.itemconfigure(wid, width=e.width))
        lc.bind("<Enter>", lambda e: lc.bind_all("<MouseWheel>", lambda ev: lc.yview_scroll(int(-ev.delta / 120), "units")))
        lc.bind("<Leave>", lambda e: lc.unbind_all("<MouseWheel>"))
        left.columnconfigure(0, weight=1)
        self._build_recording(left, 0)
        self._build_camera(left, 1)
        self._build_status(left, 2)

        # RIGHT: video + log
        right = ttk.Frame(body)
        right.rowconfigure(0, weight=1)
        right.rowconfigure(1, weight=0)
        right.columnconfigure(0, weight=1)
        body.add(right, weight=1)
        self._build_video(right)
        bottom = ttk.Frame(right)
        bottom.grid(row=1, column=0, sticky="nsew")
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=0)
        bottom.rowconfigure(0, weight=1)
        self._build_log(bottom)
        self._build_datasets(bottom)

    def _build_recording(self, parent, row):
        f = ttk.LabelFrame(parent, text="Recording", padding=(8, 6))
        f.grid(row=row, column=0, sticky="ew", pady=(0, 4))
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        self.rec_btn = tk.Button(f, text="●  START RECORDING", font=self.F_BIG, bg=REC_GREEN, fg="white",
                                 activebackground="#38b25d", activeforeground="white", disabledforeground="#9fd3ae",
                                 relief="flat", bd=0, height=2,
                                 command=self._on_start)
        self.rec_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)
        self.stop_btn = tk.Button(f, text="■  STOP RECORDING", font=self.F_BIG, bg=REC_RED, fg="white",
                                  activebackground="#d6473a", activeforeground="white", disabledforeground="#e3a29b",
                                  relief="flat", bd=0, height=2,
                                  command=self._on_stop)
        self.stop_btn.grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=2)
        opt = ttk.Frame(f)
        opt.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 2))
        ttk.Label(opt, text="Start after").pack(side=tk.LEFT)
        self.delay_var = tk.StringVar(value=str(self.cfg["delay_s"]))
        ttk.Combobox(opt, textvariable=self.delay_var, values=["0", "3", "5", "10"], width=4, state="readonly").pack(side=tk.LEFT, padx=4)
        ttk.Label(opt, text="s   (time to pick the rig up)").pack(side=tk.LEFT)
        self.delay_var.trace_add("write", lambda *_: self._set_cfg(delay_s=int(self.delay_var.get() or 0)))
        opt2 = ttk.Frame(f)
        opt2.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(2, 2))
        ttk.Label(opt2, text="Save at").pack(side=tk.LEFT)
        self.rate_var = tk.StringVar(value=str(int(self.cfg.get("record_fps", 30))))
        rc = ttk.Combobox(opt2, textvariable=self.rate_var, values=["30", "60"], width=4, state="readonly")
        rc.pack(side=tk.LEFT, padx=4)
        rc.bind("<<ComboboxSelected>>", lambda _e: self._on_rate())
        ttk.Label(opt2, text="frames/s   (60 = every camera frame; better for fast motion, 2x the disk)").pack(side=tk.LEFT)
        opt3 = ttk.Frame(f)
        opt3.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(2, 2))
        self.imu_var = tk.BooleanVar(value=bool(self.cfg.get("imu", True)))
        ttk.Checkbutton(opt3, text="Record the IMU (Cube Orange on the Jetson's USB)", variable=self.imu_var,
                        command=self._on_imu_toggle).pack(side=tk.LEFT)
        self.rows = {}
        grid = ttk.Frame(f)
        grid.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        grid.columnconfigure(0, weight=1)
        for i, (name, key) in enumerate([("State", "state"), ("Elapsed", "elapsed"), ("Frames saved", "frames"),
                                         ("Rate", "rate"), ("IMU rows", "imurows"), ("Size on Jetson", "size"), ("Free disk (Jetson)", "free")]):
            self._row(grid, i, name, key)

    def _row(self, parent, i, name, key, width=18):
        bg = "ROW_EVEN" if i % 2 else "ROW_ODD"
        n = tk.Label(parent, text=name, anchor="w", font=self.F_UI, width=22)
        self.reg(n, bg=bg, fg="HDR_FG")
        n.grid(row=i, column=0, sticky="ew", padx=(2, 4), pady=1)
        v = tk.StringVar(value="-")
        val = tk.Label(parent, textvariable=v, font=self.F_MONO_V, anchor="e", width=width)
        self.reg(val, bg=bg, fg="RO_FG")
        val.grid(row=i, column=1, sticky="e", pady=1)
        self.rows[key] = (v, val, n, bg)

    def _paint_rows(self):
        pass

    def _build_camera(self, parent, row):
        f = ttk.LabelFrame(parent, text="Camera (applies to the Jetson while it runs)", padding=(8, 6))
        f.grid(row=row, column=0, sticky="ew", pady=(0, 4))
        f.columnconfigure(1, weight=1)
        self.exp_var = tk.DoubleVar(value=float(self.cfg["exposure_ms"]))
        self.gain_var = tk.DoubleVar(value=float(self.cfg["gain"]))
        self.exp_txt = tk.StringVar(value="%.1f ms" % self.exp_var.get())
        self.gain_txt = tk.StringVar(value=str(int(self.gain_var.get())))
        for r, (name, var, txt, lo, hi) in enumerate([("Exposure", self.exp_var, self.exp_txt, 0.2, 20.0),
                                                      ("Gain", self.gain_var, self.gain_txt, 1, 40)]):
            ttk.Label(f, text=name).grid(row=r, column=0, sticky="w", pady=3)
            sc = ttk.Scale(f, from_=lo, to=hi, variable=var, orient=tk.HORIZONTAL)
            sc.grid(row=r, column=1, sticky="ew", padx=8)
            lab = tk.Label(f, textvariable=txt, font=self.F_MONO_V, width=9, anchor="e")
            self.reg(lab, bg="PANEL_BG", fg="PARAM_FG")
            lab.grid(row=r, column=2)
            if name == "Exposure":
                sc.configure(command=lambda v: self.exp_txt.set("%.1f ms" % float(v)))
                sc.bind("<ButtonRelease-1>", lambda e: self._send_exposure())
            else:
                sc.configure(command=lambda v: self.gain_txt.set(str(int(float(v)))))
                sc.bind("<ButtonRelease-1>", lambda e: self._send_gain())
        hint = tk.Label(f, text="Short exposure = less motion blur, but darker. Gain adds noise. Judge the picture by eye.",
                        font=self.F_UI, anchor="w", justify="left", wraplength=int(520 * self._scale))
        self.reg(hint, bg="PANEL_BG", fg="SEC_FG")
        hint.grid(row=2, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def _build_status(self, parent, row):
        f = ttk.LabelFrame(parent, text="Status", padding=(8, 6))
        f.grid(row=row, column=0, sticky="ew", pady=(0, 4))
        f.columnconfigure(0, weight=1)
        for i, (name, key) in enumerate([("Jetson link", "link"), ("SSH login", "ssh"), ("Capture mode", "mode"),
                                         ("Camera fps", "fps"), ("Brightness / clipped", "bright"), ("IMU (Cube Orange)", "imu"),
                                         ("Last recording", "last")]):
            self._row(f, i, name, key, width=26)
        bar = ttk.Frame(f)
        bar.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(bar, text="Start / restart capture on Jetson", command=self._start_jetson).pack(side=tk.LEFT)
        ttk.Button(bar, text="Set up Jetson login (once)", command=self._setup_login).pack(side=tk.LEFT, padx=6)

    def _build_datasets(self, parent):
        f = ttk.LabelFrame(parent, text="Recordings on the Jetson", padding=(8, 6))
        f.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        f.columnconfigure(0, weight=1)
        cols = ("name", "frames", "dur", "mb", "pc")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=4, selectmode="browse")
        for c, t, w in (("name", "Name", 150), ("frames", "Frames", 70), ("dur", "Seconds", 70), ("mb", "MB", 60), ("pc", "On this PC", 90)):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=int(w * self._scale), anchor="w" if c == "name" else "e")
        tsb = ttk.Scrollbar(f, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=tsb.set)
        self.tree.grid(row=0, column=0, columnspan=3, sticky="ew")
        tsb.grid(row=0, column=3, sticky="ns")
        bar = ttk.Frame(f)
        bar.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        ttk.Button(bar, text="Refresh", command=self._refresh_sets).pack(side=tk.LEFT)
        ttk.Button(bar, text="Copy selected to this PC", command=self._copy_selected).pack(side=tk.LEFT, padx=6)
        ttk.Button(bar, text="Open folder on PC", command=self._open_dest).pack(side=tk.LEFT)
        ttk.Button(bar, text="Change PC folder...", command=self._pick_dest).pack(side=tk.LEFT, padx=6)
        self.dest_var = tk.StringVar(value=self.cfg["dest_dir"])
        tk.Label(f, textvariable=self.dest_var, anchor="w", font=self.F_UI).grid(row=2, column=0, columnspan=3, sticky="w", pady=(4, 0))
        self.reg(f.grid_slaves(row=2, column=0)[0], bg="PANEL_BG", fg="SEC_FG")

    def _build_video(self, parent):
        outer = ttk.LabelFrame(parent, text="Video", padding=2)
        outer.grid(row=0, column=0, sticky="nsew")
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(1, weight=1)
        outer.rowconfigure(3, weight=1)
        self.vlabel = [ttk.Label(outer, text="", font=("Consolas", 9), anchor="center") for _ in range(2)]
        self.canvas = [tk.Canvas(outer, bg="#1a1a1a", highlightthickness=0, width=640, height=240) for _ in range(2)]
        self.vlabel[0].grid(row=0, column=0, sticky="ew", pady=(2, 1))
        self.canvas[0].grid(row=1, column=0, sticky="nsew", pady=(0, 2))
        self.vlabel[1].grid(row=2, column=0, sticky="ew", pady=(2, 1))
        self.canvas[1].grid(row=3, column=0, sticky="nsew", pady=(2, 0))
        # record badge + big countdown (drawn on the boresight canvas), like jvp-ui's UART chip
        c = self.canvas[0]
        self.badge_bg = c.create_rectangle(0, 0, 0, 0, fill="#ffdd33", outline="#000000", state="hidden")
        self.badge = c.create_text(14, 12, text="", anchor="nw", font=("Consolas", 14, "bold"), fill="#b00000", state="hidden")
        self.count_txt = c.create_text(0, 0, text="", font=("Segoe UI", 120, "bold"), fill="#ffdd33", state="hidden")
        self.msg_txt = c.create_text(0, 0, text="", font=("Segoe UI", 14), fill="#bbbbbb", justify="center")
        self.msg2 = self.canvas[1].create_text(0, 0, text="", font=("Segoe UI", 14), fill="#bbbbbb", justify="center")

    def _build_log(self, parent):
        f = ttk.LabelFrame(parent, text="Log", padding=6)
        f.grid(row=0, column=0, sticky="nsew")
        f.rowconfigure(0, weight=1)
        f.columnconfigure(0, weight=1)
        self.logbox = tk.Text(f, height=9, state=tk.DISABLED, font=self.F_MONO, relief="solid", bd=1, wrap=tk.WORD)
        self.reg(self.logbox, bg="LOG_BG", fg="LOG_FG")
        sb = ttk.Scrollbar(f, orient=tk.VERTICAL, command=self.logbox.yview)
        self.logbox.configure(yscrollcommand=sb.set)
        self.logbox.grid(row=0, column=0, sticky="nsew")
        sb.grid(row=0, column=1, sticky="ns")

    # -- small helpers ----------------------------------------------------------------------------------
    def _set_cfg(self, **kw):
        self.cfg.update(kw)
        self._save_cfg()

    def log(self, text):
        def _append():
            try:
                self.logbox.configure(state=tk.NORMAL)
                self.logbox.insert(tk.END, time.strftime("%H:%M:%S  ") + str(text) + "\n")
                self.logbox.see(tk.END)
                self.logbox.configure(state=tk.DISABLED)
            except tk.TclError:
                pass
        try:
            self.root.after(0, _append)
        except RuntimeError:
            pass

    def ip(self):
        return self.ip_var.get().strip()

    def cmd_port(self):
        try:
            return int(self.cport_var.get())
        except ValueError:
            return 5010

    def mono(self):
        return self.cams_var.get().startswith("Boresight")

    def send(self, cmd, timeout=2.0):
        return udp_send(self.ip(), self.cmd_port(), cmd, timeout)

    # -- connection -----------------------------------------------------------------------------------------
    def _apply_connection(self, first=False):
        self._set_cfg(ip=self.ip(), user=self.user_var.get().strip(), cameras=1 if self.mono() else 2,
                      video_port=int(self.vport_var.get() or 5000), cmd_port=self.cmd_port())
        vp = int(self.vport_var.get() or 5000)
        w, h = (MONO_PREVIEW if self.mono() else STEREO_PREVIEW)[:2]
        self.rx[0].configure(vp, w, h)
        outer = self.canvas[0].master
        outer.rowconfigure(3, weight=0 if self.mono() else 1)      # the hidden camera must not reserve half the panel
        if self.mono():
            self.rx[1].standby()
            self.vlabel[1].grid_remove()
            self.canvas[1].grid_remove()
            self.vstatus_lbl2.pack_forget()
            self.vlabel[0].configure(text="Boresight 6 mm   (UDP %d)" % vp)
        else:
            self.rx[1].configure(vp + 2, w, h)
            self.vlabel[1].grid()
            self.canvas[1].grid()
            if not self.vstatus_lbl2.winfo_ismapped():
                self.vstatus_lbl2.pack(side=tk.LEFT, padx=4)
            self.vlabel[0].configure(text="Boresight 6 mm   (UDP %d)" % vp)
            self.vlabel[1].configure(text="Depression 8 mm   (UDP %d)" % (vp + 2))
        self.pv_size = [None, None]
        self.log("connection: Jetson %s, %s, video UDP %d, control UDP %d" % (
            self.ip(), "mono (boresight)" if self.mono() else "both cameras", vp, self.cmd_port()))
        threading.Thread(target=self._check_ssh, daemon=True).start()
        threading.Thread(target=self._refresh_sets, daemon=True).start()
        if not first and self.cfg.get("autostart", True):
            self._auto_connect()

    def _ssh(self, remote, stdin_data=None, timeout=40, capture=True):
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new",
               "%s@%s" % (self.user_var.get().strip(), self.ip()), remote]
        return subprocess.run(cmd, input=stdin_data, stdout=subprocess.PIPE if capture else None,
                              stderr=subprocess.PIPE if capture else None, timeout=timeout, creationflags=NO_WINDOW)

    def _check_ssh(self):
        try:
            r = self._ssh("echo ok", timeout=15)
            self.ssh_ok = (r.returncode == 0 and b"ok" in r.stdout)
            if not self.ssh_ok:
                err = r.stderr.decode(errors="ignore").strip().splitlines()
                self.log("ssh login failed: %s" % (err[-1] if err else "no key login yet - use 'Set up Jetson login'"))
        except FileNotFoundError:
            self.ssh_ok = False
            self.log("ssh.exe not found - install the Windows 'OpenSSH Client' optional feature")
        except Exception as e:
            self.ssh_ok = False
            self.log("ssh check failed: %s" % e)

    def _auto_connect(self):
        """Make sure a capture program of the right version and mode is running on the Jetson."""
        def run():
            st = udp_send(self.ip(), self.cmd_port(), "status", 2.0)
            want = 1 if self.mono() else 2
            if st and st.get("api", 1) >= REQUIRED_API and st.get("cameras", 2) == want:
                return
            if st and st.get("recording"):
                self.log("a recording is running on the Jetson - not restarting the capture program")
                return
            if st:
                self.log("capture program on the Jetson is %s - restarting it" % (
                    "outdated" if st.get("api", 1) < REQUIRED_API else "in the other camera mode"))
            else:
                self.log("no capture program answering on the Jetson - starting it ...")
            self._start_jetson_blocking()
        threading.Thread(target=run, daemon=True).start()

    def _start_jetson(self):
        if self.busy:
            return
        threading.Thread(target=self._start_jetson_blocking, daemon=True).start()

    def _start_jetson_blocking(self):
        if self.busy:
            return
        self.busy = True
        try:
            script = resource("stereo_capture.py")
            if not script:
                self.log("!! stereo_capture.py is missing next to the app - cannot start the Jetson side")
                return
            if udp_send(self.ip(), self.cmd_port(), "status", 1.0):
                self.log("stopping the running capture program ...")
                udp_send(self.ip(), self.cmd_port(), "quit", 1.0)
                time.sleep(3.0)
            self.log("uploading the capture program to the Jetson ...")
            with open(script, "rb") as fh:
                r = self._ssh("mkdir -p ~/stereo_capture ~/recordings && cat > ~/stereo_capture/stereo_capture.py",
                              stdin_data=fh.read(), timeout=60)
            if r.returncode != 0:
                err = r.stderr.decode(errors="ignore").strip().splitlines()
                self.ssh_ok = False
                self.log("!! upload failed: %s" % (err[-1] if err else r.returncode))
                self.log("   -> press 'Set up Jetson login (once)' and type the Jetson password in the window that opens")
                return
            self.ssh_ok = True
            mono = self.mono()
            w, h, fps, kbps = MONO_PREVIEW if mono else STEREO_PREVIEW
            me = local_ip_towards(self.ip())
            vp = int(self.vport_var.get() or 5000)
            exp10 = max(1, int(round(float(self.exp_var.get()) * 10)))
            remote = ("cd ~/stereo_capture && D0=/dev/boresight; [ -e $D0 ] || D0=/dev/video0; "
                      "D1=/dev/depression; [ -e $D1 ] || D1=/dev/video2; "
                      "(nohup setsid python3 stereo_capture.py --left $D0 --right $D1 --cameras %d --dest %s "
                      "--left-port %d --right-port %d --control-port %d --preview-width %d --preview-fps %d --stream-kbps %d "
                      "--set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=%d --set-ctrl gain=%d --record-fps %d "
                      "--imu-port auto --imu-rate 100 %s"
                      "> live.log 2>&1 < /dev/null &); sleep 1; echo started"
                      % (1 if mono else 2, me, vp, vp + 2, self.cmd_port(), w, fps, kbps, exp10, int(self.gain_var.get()),
                         int(self.rate_var.get()), "" if self.imu_var.get() else "--no-imu-record "))
            r = self._ssh(remote, timeout=40)
            if b"started" in r.stdout:
                self.log("capture program started on the Jetson (streaming to %s) - video appears in a few seconds" % me)
            else:
                self.log("!! could not start the capture program: %s" % r.stderr.decode(errors="ignore").strip()[-200:])
        except subprocess.TimeoutExpired:
            self.log("!! ssh timed out")
        except Exception as e:
            self.log("!! start failed: %s" % e)
        finally:
            self.busy = False

    def _setup_login(self):
        """One-time key install: opens a console where the user types the Jetson password (never handled by this app)."""
        user, ip = self.user_var.get().strip(), self.ip()
        bat = os.path.join(tempfile.gettempdir(), "rigcapture_setup_login.bat")
        with open(bat, "w") as f:
            f.write("@echo off\r\n"
                    "echo Setting up password-less ssh login to %s@%s\r\n"
                    "if not exist \"%%USERPROFILE%%\\.ssh\" mkdir \"%%USERPROFILE%%\\.ssh\"\r\n"
                    "if not exist \"%%USERPROFILE%%\\.ssh\\id_ed25519\" ssh-keygen -t ed25519 -N \"\" -f \"%%USERPROFILE%%\\.ssh\\id_ed25519\"\r\n"
                    "echo.\r\necho Type the Jetson password when asked (it is not shown):\r\n"
                    "type \"%%USERPROFILE%%\\.ssh\\id_ed25519.pub\" | ssh -o StrictHostKeyChecking=accept-new %s@%s "
                    "\"mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys\"\r\n"
                    "echo.\r\necho Done. You can close this window and go back to Rig Capture.\r\npause\r\n" % (user, ip, user, ip))
        subprocess.Popen(["cmd", "/c", bat], creationflags=getattr(subprocess, "CREATE_NEW_CONSOLE", 0))
        self.log("a console window opened: type the Jetson password there, then press 'Apply' here")

    # -- recording --------------------------------------------------------------------------------------------------
    def imu_text(self, imu):
        """(text, colour) describing the Cube / IMU state reported by the Jetson."""
        if not imu or not imu.get("enabled", False):
            return "disabled", None
        state = imu.get("state")
        if state == "connected":
            txt = "%.0f Hz  (max gap %d ms)" % (imu.get("hz", 0), imu.get("gap_ms", 0))
            if imu.get("armed"):
                txt += "  ARMED"
            return txt, "#3ec16a"
        if state == "no permission":
            return "no access (dialout)", "#ff6b6b"
        return "Cube not found", "#e0a800"

    def _on_start(self):
        if self._countdown_job is not None:
            return
        st = self.status if self.status and time.time() - self.status_t < STATUS_STALE_S else None
        if self.imu_var.get() and st is not None and not self.args.test_rec and (st.get("imu") or {}).get("state") != "connected":
            txt = self.imu_text(st.get("imu"))[0]
            if not messagebox.askyesno("IMU not connected",
                                       "The Cube Orange IMU is not available on the Jetson (%s).\n\n"
                                       "Record the camera only, without the IMU?" % txt):
                return
        try:
            delay = int(self.delay_var.get())
        except ValueError:
            delay = 0
        if delay <= 0:
            self._do_start()
            return
        self.countdown_left = delay
        self._count_step()

    def _count_step(self):
        if self.countdown_left <= 0:
            self._countdown_job = None
            self.canvas[0].itemconfigure(self.count_txt, state="hidden")
            self._do_start()
            return
        c = self.canvas[0]
        c.coords(self.count_txt, c.winfo_width() // 2, c.winfo_height() // 2)
        c.itemconfigure(self.count_txt, text=str(self.countdown_left), state="normal")
        c.tag_raise(self.count_txt)
        self.countdown_left -= 1
        self._countdown_job = self.root.after(1000, self._count_step)

    def _do_start(self):
        def run():
            r = self.send("rec", 3.0)
            if r is None:
                self.log("!! no reply from the Jetson - is the capture program running? ('Start / restart capture')")
            elif r.get("error"):
                self.log("!! %s" % r["error"])
            elif r.get("recording"):
                self.log("recording started -> %s" % r.get("dataset"))
                self.status, self.status_t = r, time.time()
            else:
                self.log("!! recording did not start (disk space?)")
        threading.Thread(target=run, daemon=True).start()

    def _on_stop(self):
        if self._countdown_job is not None:
            self.root.after_cancel(self._countdown_job)
            self._countdown_job = None
            self.canvas[0].itemconfigure(self.count_txt, state="hidden")
            self.log("countdown cancelled")
            return

        def run():
            r = self.send("stop", 15.0)
            if r is None:
                self.log("!! no reply from the Jetson")
                return
            self.status, self.status_t = r, time.time()
            self.log("recording stopped - last: %s" % (r.get("last_dataset") or "?"))
            self._refresh_sets()
        threading.Thread(target=run, daemon=True).start()

    def _on_imu_toggle(self):
        on = bool(self.imu_var.get())
        self._set_cfg(imu=on)

        def run():
            r = self.send("imurec %d" % (1 if on else 0), 2.0)
            if r and r.get("error"):
                self.log("!! %s" % r["error"])
                self.root.after(0, lambda: self.imu_var.set(bool((r.get("imu") or {}).get("record", True))))
            elif r:
                self.log("IMU recording %s" % ("on" if on else "off"))
        threading.Thread(target=run, daemon=True).start()

    def _on_rate(self):
        n = int(self.rate_var.get())
        self._set_cfg(record_fps=n)

        def run():
            r = self.send("rate %d" % n, 2.0)
            if r and r.get("error"):
                self.log("!! %s" % r["error"])
                self.root.after(0, lambda: self.rate_var.set(str(int(r.get("record_fps") or 30))))
            elif r:
                self.log("saving %d frames/s" % n)
        threading.Thread(target=run, daemon=True).start()

    def _send_exposure(self):
        ms = round(float(self.exp_var.get()), 1)
        self._set_cfg(exposure_ms=ms)
        threading.Thread(target=lambda: (self.send("exp %d" % max(1, round(ms * 10))), self.log("exposure %.1f ms" % ms)),
                         daemon=True).start()

    def _test_exposure(self):
        """Self-test of the slider path: 6.0 ms -> read back from the Jetson -> 4.0 ms again."""
        def run():
            self.exp_var.set(6.0)
            self._send_exposure()
            time.sleep(1.5)
            st = self.send("status") or {}
            self.log("test: exposure read back from the Jetson = %s (100 us units, expected 60)" % st.get("exposure_100us"))
            self.exp_var.set(4.0)
            self._send_exposure()
            time.sleep(1.0)
            st = self.send("status") or {}
            self.log("test: exposure restored = %s (expected 40)" % st.get("exposure_100us"))
        threading.Thread(target=run, daemon=True).start()

    def _send_gain(self):
        g = int(float(self.gain_var.get()))
        self._set_cfg(gain=g)
        threading.Thread(target=lambda: (self.send("gain %d" % g), self.log("gain %d" % g)), daemon=True).start()

    # -- recordings list / copy ----------------------------------------------------------------------------------------
    def _refresh_sets(self):
        def run():
            try:
                r = self._ssh("python3 -", stdin_data=REMOTE_LIST_PY.encode(), timeout=30)
                items = json.loads(r.stdout.decode()) if r.returncode == 0 else None
            except Exception:
                items = None
            if items is None:
                self.log("could not read the recordings list (ssh login set up?)")
                return
            dest = self.cfg["dest_dir"]
            self.local_sets = set(os.listdir(dest)) if os.path.isdir(dest) else set()
            self.root.after(0, lambda: self._fill_tree(items))
        threading.Thread(target=run, daemon=True).start()

    def _fill_tree(self, items):
        sel = self.tree.selection()
        self.tree.delete(*self.tree.get_children())
        for it in items:
            state = "" if it["complete"] else "  (incomplete)"
            self.tree.insert("", "end", iid=it["name"], values=(
                it["name"] + state, it["frames"] if it["frames"] is not None else "-",
                ("%.0f" % it["dur"]) if it["dur"] else "-", it["mb"], "yes" if it["name"] in self.local_sets else "-"))
        if sel and self.tree.exists(sel[0]):
            self.tree.selection_set(sel[0])
        elif items:
            self.tree.selection_set(items[-1]["name"])

    def _copy_selected(self):
        sel = self.tree.selection()
        if not sel:
            self.log("select a recording in the list first")
            return
        threading.Thread(target=lambda: self._copy_dataset(sel[0]), daemon=True).start()

    def _copy_dataset(self, name):
        """Copy ~/recordings/<name> from the Jetson into the PC folder (blocking; run in a thread)."""
        dest = self.cfg["dest_dir"]
        os.makedirs(dest, exist_ok=True)
        self.log("copying %s to %s ..." % (name, dest))
        try:
            p1 = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "%s@%s" % (self.user_var.get().strip(), self.ip()),
                                   "tar cf - -C ~/recordings %s" % shlex.quote(name)],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, creationflags=NO_WINDOW)
            p2 = subprocess.Popen(["tar", "xf", "-", "-C", dest], stdin=p1.stdout, creationflags=NO_WINDOW)
            p1.stdout.close()
            p2.wait()
            p1.wait()
            ok = p1.returncode == 0 and p2.returncode == 0 and os.path.isdir(os.path.join(dest, name))
            n = 0
            if ok:
                d = os.path.join(dest, name, "mav0", "cam0", "data")
                if not os.path.isdir(d):
                    d = os.path.join(dest, name, "cam0", "data")
                n = len(os.listdir(d)) if os.path.isdir(d) else 0
            self.log(("copied %s: %d images -> %s" % (name, n, os.path.join(dest, name))) if ok else
                     "!! copy of %s failed" % name)
        except Exception as e:
            self.log("!! copy failed: %s" % e)
        self._refresh_sets()

    def _open_dest(self):
        d = self.cfg["dest_dir"]
        os.makedirs(d, exist_ok=True)
        try:
            os.startfile(d)
        except AttributeError:
            subprocess.Popen(["xdg-open", d])

    def _pick_dest(self):
        d = filedialog.askdirectory(initialdir=self.cfg["dest_dir"], title="Folder for copied recordings")
        if d:
            self._set_cfg(dest_dir=d)
            self.dest_var.set(d)

    # -- video + status polling --------------------------------------------------------------------------------------------
    def _frame_cb(self, i, raw, w, h):
        now = time.monotonic()
        self.frame_t[i] = now
        ft = self.fps_times[i]
        ft.append(now)
        del ft[:-30]
        try:
            self.frame_q[i].put_nowait((raw, w, h))
        except queue.Full:
            pass

    def _poll_frames(self):
        for i in range(2):
            item = None
            while True:
                try:
                    item = self.frame_q[i].get_nowait()
                except queue.Empty:
                    break
            if item is None:
                continue
            raw, fw, fh = item
            if len(raw) != fw * fh * 3:
                continue
            cv = self.canvas[i]
            cw, ch = max(1, cv.winfo_width()), max(1, cv.winfo_height())
            sc = min(cw / fw, ch / fh)
            dw, dh = max(1, int(fw * sc)), max(1, int(fh * sc))
            im = Image.frombuffer("RGB", (fw, fh), raw, "raw", "RGB", 0, 1)
            if (dw, dh) != (fw, fh):
                im = im.resize((dw, dh), Image.BILINEAR)
            self.photo[i] = ImageTk.PhotoImage(im)
            if self.img_id[i] is None:
                self.img_id[i] = cv.create_image(cw // 2, ch // 2, image=self.photo[i])
                cv.tag_lower(self.img_id[i])
            else:
                cv.coords(self.img_id[i], cw // 2, ch // 2)
                cv.itemconfig(self.img_id[i], image=self.photo[i])
        self.root.after(15, self._poll_frames)

    def _poll_status(self):
        while True:
            r = udp_send(self.ip(), self.cmd_port(), "status", 0.5)
            if r:
                self.status, self.status_t = r, time.time()
            time.sleep(0.25)

    def _fps(self, i):
        ft = self.fps_times[i]
        if len(ft) < 5 or time.monotonic() - ft[-1] > 2:
            return 0.0
        return (len(ft) - 1) / max(1e-6, ft[-1] - ft[0])

    # -- the 5 Hz screen update ----------------------------------------------------------------------------------------------------
    def _set(self, key, text, fg=None):
        v, lab, _n, _bg = self.rows[key]
        v.set(text)
        lab.configure(fg=fg or THEMES[self.theme_var.get()]["RO_FG"])

    def _tick(self):
        st = self.status if self.status and time.time() - self.status_t < STATUS_STALE_S else None
        for i in range(2):
            live = time.monotonic() - self.frame_t[i] < VIDEO_ACTIVE_TIMEOUT and self.frame_t[i] > 0
            m = self.canvas[i]
            txt = self.msg_txt if i == 0 else self.msg2
            m.coords(txt, m.winfo_width() // 2, m.winfo_height() // 2)
            if live:
                m.itemconfigure(txt, text="")
            else:
                m.itemconfigure(txt, text="waiting for video ...\n(is the capture running on the Jetson?)" if st is None else
                                "waiting for video ...\n(firewall: allow gst-launch-1.0.exe for UDP?)")
        if st:
            mono = st.get("cameras", 2) == 1
            pv = st.get("preview")
            if pv and self.pv_size[0] != tuple(pv[:2]):
                self.pv_size[0] = tuple(pv[:2])
                vp = int(self.vport_var.get() or 5000)
                if tuple(pv[:2]) != (self.rx[0].width, self.rx[0].height):
                    self.rx[0].configure(vp, pv[0], pv[1])
                    if not self.mono():
                        self.rx[1].configure(vp + 2, pv[0], pv[1])
                    self.log("preview size on the Jetson is %dx%d - receiver adjusted" % (pv[0], pv[1]))
            self._set("link", "connected (%s)" % st.get("mode", "?"), "#3ec16a")
            self._set("mode", "%s  %s" % (st.get("mode", "?"), " + ".join(st.get("roles", []))))
            self._set("fps", " / ".join("%.0f" % f for f in st.get("fps", [])) + "  (camera)")
            imu = st.get("imu")
            txt, col = self.imu_text(imu)
            self._set("imu", txt, col)
            if imu and bool(imu.get("record", True)) != bool(self.imu_var.get()) and not st.get("recording") \
                    and time.time() - getattr(self, "_imu_sync_t", 0) > 3.0:
                self._imu_sync_t = time.time()           # keep the Jetson in step with the checkbox
                want = 1 if self.imu_var.get() else 0
                threading.Thread(target=lambda: self.send("imurec %d" % want, 2.0), daemon=True).start()
            self._set("bright", "  ".join("%d/255  %.1f%%" % (m_, c_) for m_, c_ in zip(st.get("mean", []), st.get("clip_pct", []))))
            ex = st.get("exposure_100us")
            if st.get("recording"):
                el = st.get("elapsed_s", 0)
                self._set("state", "RECORDING", "#ff6b6b")
                self._set("elapsed", "%02d:%02d" % (el // 60, el % 60))
                n = st.get("pairs", 0)
                self._set("frames", str(n))
                self._set("imurows", str((st.get("imu") or {}).get("rows", 0)) if (st.get("imu") or {}).get("record") else "off")
                self._set("rate", "%.0f / s" % (n / el) if el > 1 else "-")
                self._set("size", "~%.0f MB" % (n * (0.35 if mono else 0.7)))
                if el > 1 and st.get("record_fps"):
                    self._set("rate", "%.0f / s  (saving %d)" % (n / el, st["record_fps"]))
            else:
                self._set("state", "idle" if self._countdown_job is None else "starting ...")
                for k in ("elapsed", "frames", "rate", "imurows", "size"):
                    self._set(k, "-")
            self._set("free", "%.0f GB" % st.get("free_gb", 0))
            want = int(self.rate_var.get())
            if (not st.get("recording") and st.get("record_fps") and int(round(st["record_fps"])) != want
                    and time.time() - getattr(self, "_rate_sync_t", 0) > 3.0):
                self._rate_sync_t = time.time()
                threading.Thread(target=lambda: self.send("rate %d" % want, 2.0), daemon=True).start()
            last = st.get("last_dataset")
            self._set("last", posixpath.basename(last) if last else "-")
            rec = bool(st.get("recording"))
            self.rec_btn.configure(state="disabled" if rec or self._countdown_job else "normal")
            self.stop_btn.configure(state="normal" if rec or self._countdown_job else "disabled")
            c = self.canvas[0]
            if rec:
                el = st.get("elapsed_s", 0)
                c.itemconfigure(self.badge, text="  ● REC %02d:%02d   %d frames  " % (el // 60, el % 60, st.get("pairs", 0)), state="normal")
                bb = c.bbox(self.badge)
                if bb:
                    c.coords(self.badge_bg, bb[0] - 2, bb[1] - 2, bb[2] + 2, bb[3] + 2)
                c.itemconfigure(self.badge_bg, state="normal")
                c.tag_raise(self.badge_bg)
                c.tag_raise(self.badge)
            else:
                c.itemconfigure(self.badge, state="hidden")
                c.itemconfigure(self.badge_bg, state="hidden")
        else:
            self._set("link", "no reply", "#ff6b6b")
            self.rec_btn.configure(state="disabled" if self._countdown_job else "normal")
            self.stop_btn.configure(state="normal" if self._countdown_job else "disabled")
            self.canvas[0].itemconfigure(self.badge, state="hidden")
            self.canvas[0].itemconfigure(self.badge_bg, state="hidden")
        self._set("ssh", {None: "checking ...", True: "OK", False: "key login needed"}[self.ssh_ok],
                  "#3ec16a" if self.ssh_ok else ("#ff6b6b" if self.ssh_ok is False else None))
        self.root.after(200, self._tick)

    def _shot(self):
        self.root.update()
        grab_window(self.root).save(self.args.shot)
        if self.args.test_rec:
            r = self.send("stop", 15.0)
            last = (r or {}).get("last_dataset")
            self.log("test: recording stopped -> %s" % last)
            if last:
                done = threading.Event()
                threading.Thread(target=lambda: (self._copy_dataset(posixpath.basename(last)), done.set()), daemon=True).start()
                while not done.is_set():
                    self.root.update()
                    time.sleep(0.05)
                time.sleep(2.0)
                self.root.update()
            grab_window(self.root).save(self.args.shot + ".after.png")
        self._on_close()

    def _on_close(self):
        for r in self.rx:
            r.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def grab_window(root):
    """Picture of the Tk window itself (Windows PrintWindow), so it also works when the screen is locked."""
    import ctypes
    from ctypes import wintypes
    user32, gdi32 = ctypes.windll.user32, ctypes.windll.gdi32
    hwnd = root.winfo_id()
    rect = wintypes.RECT()
    user32.GetClientRect(hwnd, ctypes.byref(rect))
    w, h = rect.right, rect.bottom

    class BIH(ctypes.Structure):
        _fields_ = [("biSize", ctypes.c_uint32), ("biWidth", ctypes.c_int32), ("biHeight", ctypes.c_int32),
                    ("biPlanes", ctypes.c_uint16), ("biBitCount", ctypes.c_uint16), ("biCompression", ctypes.c_uint32),
                    ("biSizeImage", ctypes.c_uint32), ("biXPels", ctypes.c_int32), ("biYPels", ctypes.c_int32),
                    ("biClrUsed", ctypes.c_uint32), ("biClrImportant", ctypes.c_uint32)]
    hdc = user32.GetDC(hwnd)
    mdc = gdi32.CreateCompatibleDC(hdc)
    bmp = gdi32.CreateCompatibleBitmap(hdc, w, h)
    gdi32.SelectObject(mdc, bmp)
    user32.PrintWindow(hwnd, mdc, 3)          # PW_CLIENTONLY | PW_RENDERFULLCONTENT
    buf = ctypes.create_string_buffer(w * h * 4)
    bi = BIH(40, w, -h, 1, 32, 0, 0, 0, 0, 0, 0)
    gdi32.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bi), 0)
    gdi32.DeleteObject(bmp)
    gdi32.DeleteDC(mdc)
    user32.ReleaseDC(hwnd, hdc)
    return Image.frombuffer("RGB", (w, h), buf, "raw", "BGRX", 0, 1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--jetson", help="Jetson IP (default: last used, else 192.168.0.40)")
    p.add_argument("--user", help="ssh user on the Jetson")
    p.add_argument("--video-port", type=int)
    p.add_argument("--cmd-port", type=int)
    p.add_argument("--theme", choices=list(THEMES))
    p.add_argument("--no-autostart", action="store_true", help="do not start the capture program on the Jetson by itself")
    p.add_argument("--shot", help="save a screenshot of the window to this file and exit (testing)")
    p.add_argument("--shot-after", type=float, default=10.0)
    p.add_argument("--test-rec", action="store_true", help="(testing) click START by itself before the screenshot")
    App(p.parse_args()).run()


if __name__ == "__main__":
    main()
