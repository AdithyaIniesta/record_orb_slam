"""
Self-test of the IMU rate step-down in stereo_capture.py (run in WSL/Linux, needs pymavlink for the fake Cube).

Case A: the Cube accepts up to 100 Hz (SCHED_LOOP_RATE 200)  -> a 100 Hz request is accepted at once, no step-down.
Case B: the Cube accepts up to 40 Hz (SCHED_LOOP_RATE 50)    -> the 100 Hz request is refused and the recorder steps down
        100 -> 50 -> 40 Hz by itself and ends up recording ~40 Hz.

usage:  python3 test_imu_stepdown.py /path/to/stereo_capture.py
"""
import importlib.util
import os
import pty
import sys
import tempfile
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_imu_recorder import FakeCube       # noqa: E402


def run_case(sc, max_hz, seconds=9.0):
    master, slave = pty.openpty()
    tty.setraw(master)
    cube = FakeCube(master)
    cube.max_hz = max_hz
    cube.start()
    logs = []
    rec = sc.ImuRecorder(os.ttyname(slave), 100.0, True, log=lambda s: logs.append(s))
    rec.start()
    t0 = time.time()
    while not rec.is_connected() and time.time() - t0 < 8:
        time.sleep(0.1)
    time.sleep(seconds)                        # let the step-down settle
    root = tempfile.mkdtemp(prefix="imu_step_")
    rec.start_recording(root)
    time.sleep(5.0)
    info = rec.stop_recording()
    final_rate = rec.rate_hz
    asked = sorted({round(1e6 / c[1]) for c in cube.commands if c[1] > 0 and c[0] == 27})
    denied = rec.denied
    rec.shutdown()
    cube.running = False
    return final_rate, asked, denied, info, logs


def main():
    spec = importlib.util.spec_from_file_location("stereo_capture", sys.argv[1])
    sc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sc)
    ok = {}

    rate, asked, denied, info, logs = run_case(sc, max_hz=101.0)
    print("A (Cube accepts up to 100 Hz): final request %g Hz, rates asked for IMU1: %s, denied %d, recorded %.1f Hz" % (
        rate, asked, denied, info["measured_hz"]))
    ok["A: 100 Hz accepted without any step-down"] = rate == 100.0 and denied == 0 and asked == [100]
    ok["A: about 100 Hz recorded (%.1f)" % info["measured_hz"]] = 90.0 <= info["measured_hz"] <= 106.0

    rate, asked, denied, info, logs = run_case(sc, max_hz=41.0)
    print("B (Cube accepts up to 40 Hz):  final request %g Hz, rates asked for IMU1: %s, denied %d, recorded %.1f Hz" % (
        rate, asked, denied, info["measured_hz"]))
    print("   log:", [l for l in logs if "refused" in l])
    ok["B: stepped down 100 -> 50 -> 40"] = asked == [40, 50, 100] and rate == 40.0
    ok["B: refusals were counted (%d)" % denied] = denied >= 2
    ok["B: about 40 Hz recorded (%.1f)" % info["measured_hz"]] = 35.0 <= info["measured_hz"] <= 44.0
    ok["B: the log says what happened"] = sum("refused" in l for l in logs) == 2

    bad = [k for k, v in ok.items() if not v]
    for k, v in ok.items():
        print("%-4s %s" % ("PASS" if v else "FAIL", k))
    print("\n%d/%d passed" % (len(ok) - len(bad), len(ok)))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
