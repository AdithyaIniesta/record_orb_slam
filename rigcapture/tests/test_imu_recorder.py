"""
Self-test of the stdlib IMU recorder in stereo_capture.py against a FAKE Cube on a pty (run in WSL/Linux).

The fake Cube speaks real MAVLink 2 through pymavlink (so the encoder is independent of the code under test):
heartbeat, IMU streams that start only after a SET_MESSAGE_INTERVAL request (ACCEPTED up to 40 Hz, DENIED above),
TIMESYNC replies with a known clock offset, plus injected garbage and a corrupted frame.

usage:  python3 test_imu_recorder.py /path/to/stereo_capture.py
"""
import importlib.util
import math
import os
import pty
import shutil
import sys
import tempfile
import threading
import time
import tty

from pymavlink.dialects.v20 import ardupilotmega as mav

CUBE_OFFSET_NS = 5_000_000_000          # the fake Cube's clock = this computer's CLOCK_MONOTONIC + 5 s


class FakeCube(threading.Thread):
    def __init__(self, master):
        threading.Thread.__init__(self, daemon=True)
        self.fd = master
        self.mav = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.rates = {}                          # message id -> Hz
        self.next = {}
        self.running = True
        self.commands = []                       # (msgid, interval_us) in the order received
        self.timesync_replies = 0
        self.garbage_done = False
        self.max_hz = 41.0                       # rates above this are DENIED (loop rate 50 -> ~40; loop rate 200 -> ~100)

    def send(self, m):
        os.write(self.fd, m.pack(self.mav))

    def t_boot_us(self):
        return int((time.monotonic_ns() + CUBE_OFFSET_NS) / 1000)

    def run(self):
        last_hb = 0.0
        parser = mav.MAVLink(None)
        t_start = time.time()
        while self.running:
            now = time.time()
            if now - last_hb >= 1.0:
                last_hb = now
                self.send(mav.MAVLink_heartbeat_message(1, 3, 81, 0, 4, 3))     # fixed-wing, ArduPilot, disarmed
                # a peripheral on the same link (autopilot = INVALID) must be ignored
                os.write(self.fd, mav.MAVLink_heartbeat_message(27, 8, 0, 0, 4, 3).pack(
                    mav.MAVLink(None, srcSystem=1, srcComponent=156)))
            for mid, hz in list(self.rates.items()):
                if hz <= 0:
                    continue
                if now >= self.next.get(mid, 0):
                    self.next[mid] = max(self.next.get(mid, now), now - 0.05) + 1.0 / hz
                    self.emit(mid, now - t_start)
            if not self.garbage_done and now - t_start > 3.0:
                self.garbage_done = True
                os.write(self.fd, b"\x00\xfd\xfd\xfd\x09\xff\xfe\x12garbage-bytes\xfd")           # junk with fake STX bytes
                good = mav.MAVLink_raw_imu_message(self.t_boot_us(), 0, 0, -1000, 0, 0, 0, 0, 0, 0, 0, 0).pack(self.mav)
                bad = bytearray(good)
                bad[-1] ^= 0xFF                                                                  # corrupt the CRC
                os.write(self.fd, bytes(bad))
            try:
                import select
                r, _, _ = select.select([self.fd], [], [], 0.002)
                if r:
                    data = os.read(self.fd, 4096)
                    for b in data:
                        m = parser.parse_char(bytes([b]))
                        if m is None:
                            continue
                        if m.get_type() == "COMMAND_LONG" and m.command == 511:
                            mid, interval = int(m.param1), int(m.param2)
                            self.commands.append((mid, interval))
                            hz = 0.0 if interval == 0 else 1e6 / interval
                            ok = interval == 0 or hz <= self.max_hz
                            if ok:
                                self.rates[mid] = hz if interval else {27: 2.0, 116: 2.0, 129: 2.0, 30: 4.0}.get(mid, 1.0)
                            self.send(mav.MAVLink_command_ack_message(511, 0 if ok else 2, 0, 0, 255, 191))
                        elif m.get_type() == "TIMESYNC" and m.tc1 == 0:
                            self.timesync_replies += 1
                            self.send(mav.MAVLink_timesync_message(int((time.monotonic_ns() + CUBE_OFFSET_NS)), m.ts1))
            except OSError:
                break

    def emit(self, mid, t):
        us = self.t_boot_us()
        gx, gy, gz = 0.20 * math.sin(2 * math.pi * 0.5 * t), -0.10 * math.cos(2 * math.pi * 0.3 * t), 0.05
        ax, ay, az = 0.3 * math.sin(2 * math.pi * 0.7 * t), 0.1, -9.80665 + 0.2 * math.cos(2 * math.pi * 0.4 * t)
        mg = lambda v: int(round(v / 9.80665 * 1000))
        mr = lambda v: int(round(v * 1000))
        if mid == 27:
            self.send(mav.MAVLink_raw_imu_message(us, mg(ax), mg(ay), mg(az), mr(gx), mr(gy), mr(gz), 0, 0, 0, 0, 0))
        elif mid in (116, 129):
            cls = mav.MAVLink_scaled_imu2_message if mid == 116 else mav.MAVLink_scaled_imu3_message
            self.send(cls(int(us / 1000), mg(ax), mg(ay), mg(az), mr(gx), mr(gy), mr(gz), 0, 0, 0, 0))
        elif mid == 30:
            self.send(mav.MAVLink_attitude_message(int(us / 1000), 0.1, 0.02, 1.0, gx, gy, gz))


def main():
    spec = importlib.util.spec_from_file_location("stereo_capture", sys.argv[1])
    sc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sc)

    master, slave = pty.openpty()
    tty.setraw(master)
    dev = os.ttyname(slave)
    cube = FakeCube(master)
    cube.start()
    logs = []
    rec = sc.ImuRecorder(dev, 40.0, True, log=lambda s: logs.append(s))
    rec.start()
    t0 = time.time()
    while not rec.is_connected() and time.time() - t0 < 8:
        time.sleep(0.1)
    ok = {}
    ok["connected to the fake Cube (peripheral heartbeat ignored)"] = rec.is_connected() and rec.sysid == 1 and rec.compid == 1
    time.sleep(2.0)
    root = tempfile.mkdtemp(prefix="imu_test_")
    ok["start_recording"] = rec.start_recording(root)
    time.sleep(7.0)
    info = rec.stop_recording()
    snap = rec.snapshot()
    rec.shutdown()
    time.sleep(0.5)
    cube.running = False

    print("\nlog lines from the recorder:", logs)
    print("summary:", info)
    ok["rate requests sent for IMU1-3 + attitude"] = {c[0] for c in cube.commands} >= {27, 116, 129, 30}
    ok["40 Hz request ACCEPTED (no denial logged)"] = rec.denied == 0
    ok["default rates asked back at shutdown (interval 0)"] = [c for c in cube.commands if c[1] == 0 and c[0] in (27, 116, 129, 30)].__len__() >= 4
    n0 = info["rows"][0]
    ok["about 40 Hz recorded for IMU1 (%d rows in %.1f s)" % (n0, info["duration_s"])] = 35.0 <= info["measured_hz"] <= 44.0
    ok["IMU2/IMU3 rows recorded"] = info["rows"][1] > 150 and info["rows"][2] > 150
    ok["parser survived garbage + corrupted frame (bad frames counted %d)" % rec.parser.bad] = rec.parser.bad >= 1 and n0 > 200
    ok["TIMESYNC offset ~5.000 s (got %s ms)" % info["clock_offset_timesync_ms"]] = (
        info["clock_offset_timesync_ms"] is not None and abs(info["clock_offset_timesync_ms"] - 5000.0) < 5.0)
    ok["offset used for the sample times ~5.000 s (got %s ms)" % info["clock_offset_used_ms"]] = (
        info["clock_offset_used_ms"] is not None and abs(info["clock_offset_used_ms"] - 5000.0) < 3.0)

    # file contents against the synthetic signal
    d = os.path.join(root, "mav0", "imu0")
    lines = open(os.path.join(d, "data.csv")).read().splitlines()
    ok["EuRoC header"] = lines[0].startswith("#timestamp [ns],w_RS_S_x [rad s^-1]")
    rows = [l.split(",") for l in lines[1:]]
    ok["7 columns per row"] = all(len(r) == 7 for r in rows)
    az = [float(r[6]) for r in rows]
    gz = [float(r[3]) for r in rows]
    ok["accel z ~ -9.8 +- 0.3 m/s^2 (mean %.2f)" % (sum(az) / len(az))] = abs(sum(az) / len(az) + 9.80665) < 0.3 and all(-10.2 < v < -9.4 for v in az)
    ok["gyro z = 0.05 rad/s (mean %.4f)" % (sum(gz) / len(gz))] = abs(sum(gz) / len(gz) - 0.05) < 0.002
    ts = [int(r[0]) for r in rows]
    ok["timestamps strictly increasing"] = all(b > a for a, b in zip(ts, ts[1:]))
    arr = [int(l.split(",")[2]) for l in open(os.path.join(d, "data_fc.csv")).read().splitlines()[1:]]
    ok["no sample placed after its arrival (max sample-arrival = %.3f ms)" % (max(a - b for a, b in zip(ts, arr)) / 1e6)] = all(a <= b for a, b in zip(ts, arr))
    fc = [int(l.split(",")[1]) for l in open(os.path.join(d, "data_fc.csv")).read().splitlines()[1:]]
    ok["Cube time = Jetson time + 5 s (first row diff %.3f s)" % ((fc[0] * 1000 - ts[0]) / 1e9)] = abs((fc[0] * 1000 - ts[0]) / 1e9 - 5.0) < 0.05
    for f in ("imu1/data.csv", "imu2/data.csv", "imu0/sensor.yaml", "cube/attitude.csv", "cube/timesync.csv"):
        ok["file mav0/" + f] = os.path.getsize(os.path.join(root, "mav0", f)) > 100

    width = max(len(k) for k in ok)
    for k, v in ok.items():
        print(("PASS  " if v else "FAIL  ") + k)
    shutil.rmtree(root, ignore_errors=True)
    print("\n%d/%d checks passed" % (sum(ok.values()), len(ok)))
    sys.exit(0 if all(ok.values()) else 1)


if __name__ == "__main__":
    main()
