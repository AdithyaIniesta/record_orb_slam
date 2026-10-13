#!/usr/bin/env python3
"""
replay_to_feed.py - play a RigCapture recording into the SLAM shared-memory feed as if it were the live camera.
For testing mono_live without the camera (or repeatably):

    python3 replay_to_feed.py RECORDING_DIR [--feed /dev/shm/rig_slam_cam0] [--speed 1.0] [--fps 30] [--loops 1]

Uses the real SlamFeed class of stereo_capture.py (same layout as the live program), pacing the frames by the recording's own
time stamps. Run mono_live in another terminal.
"""
import argparse
import importlib.util
import os
import sys
import time

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--feed", default="/dev/shm/rig_slam_cam0")
    ap.add_argument("--speed", type=float, default=1.0, help="1.0 = real time of the recording")
    ap.add_argument("--fps", type=float, default=30.0, help="frames per second published into the feed")
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--capture-script", default=None, help="path of stereo_capture.py (default: found next to the repo)")
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    cands = [a.capture_script, os.path.join(here, "..", "..", "rigcapture", "jetson", "stereo_capture.py"),
             os.path.expanduser("~/stereo_capture/stereo_capture.py")]
    path = next((c for c in cands if c and os.path.exists(c)), None)
    if path is None:
        sys.exit("stereo_capture.py not found (use --capture-script)")
    spec = importlib.util.spec_from_file_location("stereo_capture", path)
    sc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sc)

    stamps = [int(l) for l in open(os.path.join(a.recording, "times.txt")) if l.strip()]
    first = cv2.imread("%s/mav0/cam0/data/%d.png" % (a.recording, stamps[0]), cv2.IMREAD_GRAYSCALE)
    h, w = first.shape

    class FakeCam:                                   # what SlamFeed needs: .latest (Frame with seq, t_ns, y) and .index
        index = 0
        latest = None

    cam = FakeCam()
    feed = sc.SlamFeed(cam, a.feed, a.fps, w, h)
    feed.start()
    print("replaying %d frames (%dx%d) into %s at %.1fx real time, up to %g fps" % (len(stamps), w, h, a.feed, a.speed, a.fps))
    seq = 0
    try:
        for loop in range(a.loops):
            t_wall0 = time.monotonic()
            for ts in stamps:
                due = (ts - stamps[0]) / 1e9 / a.speed
                delay = due - (time.monotonic() - t_wall0)
                if delay > 0:
                    time.sleep(delay)
                y = cv2.imread("%s/mav0/cam0/data/%d.png" % (a.recording, ts), cv2.IMREAD_GRAYSCALE)
                seq += 1
                # a looped replay must not jump back in time for the SLAM program: keep the time increasing
                t_ns = ts + loop * (stamps[-1] - stamps[0] + 100_000_000)
                cam.latest = sc.Frame(seq, t_ns, time.monotonic_ns(), y)
            print("loop %d done (%d frames published so far)" % (loop + 1, feed.published))
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    feed.stop()


if __name__ == "__main__":
    main()
