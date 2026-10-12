#!/usr/bin/env python3
"""
cam_imu_sync.py - how well do the camera and the IMU line up in time?

    python3 cam_imu_sync.py DATASET_DIR [--calib 6mmboresight.json] [--imu imu0] [--out report.png]
    python3 cam_imu_sync.py --selftest DATASET_DIR        # camera-only data: fake gyro shifted by a known offset

DATASET_DIR is a Rig Capture recording (mav0/cam0/data/*.png, times.txt, mav0/imu0/data.csv).

Method (no extrinsics needed): the *speed of rotation* is the same in every frame of reference, so
  * from the images: track features between frames, turn them into viewing directions (with the lens calibration) and
    fit the rotation between the two views -> visual angular speed |w_cam(t)|
  * from the IMU: |w_gyro(t)| with the bias removed
and cross-correlate the two curves. The lag of the best match is the camera-IMU time offset.

    Convention: IMU time = camera time + offset   (a positive offset means the IMU stamps are LATE)

Needs a motion test: a few seconds still at the start (bias), then rotations of all kinds and some quick shakes.
The camera and the Cube must be rigidly fixed to each other.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np


def load_intrinsics(path):
    d = json.load(open(path))
    K = np.array(d["camera_matrix"], float)
    dist = np.array(d["distortion_coefficients"], float)
    return K, dist, (d["image_width"], d["image_height"])


def best_rotation(b1, b2):
    """Rotation R with R*b1 ~ b2 (Kabsch on unit bearing vectors)."""
    H = b1.T @ b2
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T


def rotation_angle(R):
    return float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def visual_rate(ds, calib, step, scale=0.5, max_frames=None, quiet=False):
    """-> (times_s, |w| rad/s) from tracked features between frame i and i+step."""
    ts = np.array([int(l) for l in open(os.path.join(ds, "times.txt"))], dtype=np.int64)
    K, dist, (w, h) = load_intrinsics(calib)
    n = len(ts) if max_frames is None else min(len(ts), max_frames)
    files = [os.path.join(ds, "mav0", "cam0", "data", "%d.png" % t) for t in ts[:n]]
    Ks = K.copy()
    Ks[:2] *= scale
    Ks[2, 2] = 1.0
    maxlen = max(1, int(1.0 / scale))
    prev, prev_pts = None, None
    t_out, w_out = [], []
    lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
    for i in range(0, n - step, 1):
        img = cv2.imread(files[i], 0)
        img2 = cv2.imread(files[i + step], 0)
        if img is None or img2 is None:
            continue
        a = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        b = cv2.resize(img2, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        p0 = cv2.goodFeaturesToTrack(a, 500, 0.01, 8)
        if p0 is None or len(p0) < 30:
            continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p0, None, **lk)
        pb, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, p1, None, **lk)       # forward-backward check
        ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(pb - p0, axis=2).ravel() < 0.7)
        if ok.sum() < 30:
            continue
        q0 = cv2.undistortPoints(p0[ok].reshape(-1, 1, 2), Ks, dist, P=None).reshape(-1, 2)
        q1 = cv2.undistortPoints(p1[ok].reshape(-1, 1, 2), Ks, dist, P=None).reshape(-1, 2)
        b0 = np.c_[q0, np.ones(len(q0))]
        b1 = np.c_[q1, np.ones(len(q1))]
        b0 /= np.linalg.norm(b0, axis=1, keepdims=True)
        b1 /= np.linalg.norm(b1, axis=1, keepdims=True)
        keep = np.ones(len(b0), bool)
        R = np.eye(3)
        for _ in range(3):                                                 # trim outliers (moving objects, translation)
            R = best_rotation(b0[keep], b1[keep])
            res = np.linalg.norm((R @ b0.T).T - b1, axis=1)
            thr = max(2.5 * np.median(res[keep]), 1e-4)
            keep = res < thr
            if keep.sum() < 20:
                break
        dt = (ts[i + step] - ts[i]) / 1e9
        t_out.append((ts[i] + ts[i + step]) / 2e9)
        w_out.append(rotation_angle(R) / dt)
        if not quiet and i % 500 == 0:
            print("  visual rotation: frame %d / %d" % (i, n), flush=True)
    return np.array(t_out), np.array(w_out)


def gyro_rate(ds, imu="imu0"):
    a = np.loadtxt(os.path.join(ds, "mav0", imu, "data.csv"), delimiter=",", comments="#")
    t = a[:, 0] / 1e9
    g = a[:, 1:4]
    mag = np.linalg.norm(g, axis=1)
    still = mag <= np.percentile(mag, 20)                                 # the quietest samples = bias ...
    bias = g[still].mean(0)
    if np.linalg.norm(bias) > 0.03:                                       # ... but only if they really were at rest
        print("note: no still period in this recording (quietest samples move at %.2f rad/s): gyro bias NOT removed;"
              " start recordings with a few seconds at rest" % np.linalg.norm(bias))
        bias = np.zeros(3)
    return t, np.linalg.norm(g - bias, axis=1), bias


def smooth(x, n):
    return np.convolve(x, np.ones(n) / n, mode="same") if n > 1 else x


def estimate_offset(tv, wv, tg, wg, max_lag=0.4, grid_hz=200.0):
    """Lag (s) so that wv(t) ~ scale * wg(t + lag); returns (lag, corr, scale, grid arrays)."""
    lo = max(tv[0], tg[0]) + max_lag
    hi = min(tv[-1], tg[-1]) - max_lag
    if hi - lo < 3.0:
        raise RuntimeError("camera and IMU overlap by only %.1f s" % max(0.0, hi - lo))
    t = np.arange(lo, hi, 1.0 / grid_hz)
    v = smooth(np.interp(t, tv, wv), 7)
    lags = np.arange(-max_lag, max_lag + 1e-9, 0.001)
    corr = np.empty(len(lags))
    vz = (v - v.mean()) / (v.std() + 1e-12)
    for k, lag in enumerate(lags):
        g = smooth(np.interp(t + lag, tg, wg), 7)
        gz = (g - g.mean()) / (g.std() + 1e-12)
        corr[k] = float((vz * gz).mean())
    k = int(np.argmax(corr))
    lag = lags[k]
    if 0 < k < len(lags) - 1:                                             # parabolic refinement
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            lag += 0.5 * (y0 - y2) / den * 0.001
    g = smooth(np.interp(t + lag, tg, wg), 7)
    scale = float((v * g).sum() / ((g * g).sum() + 1e-12))
    return lag, float(corr.max()), scale, (t, v, g, lags, corr)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("--calib", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                                    "jetson-tracking-perception", "calibration", "green_mount", "boresight",
                                                    "6mmboresight.json"))
    ap.add_argument("--imu", default="imu0")
    ap.add_argument("--step", type=int, default=0, help="frame step for the rotation estimate (default: ~33 ms)")
    ap.add_argument("--out", default="cam_imu_sync.png")
    ap.add_argument("--selftest", action="store_true", help="no IMU needed: fake gyro = visual rate shifted by 53 ms + noise")
    ap.add_argument("--max-frames", type=int)
    a = ap.parse_args()

    ts = np.array([int(l) for l in open(os.path.join(a.dataset, "times.txt"))], dtype=np.int64)
    fps = (len(ts) - 1) / ((ts[-1] - ts[0]) / 1e9)
    step = a.step or max(1, int(round(fps * 0.033)))
    print("dataset %s: %d frames, %.1f fps, rotation measured over %d frame(s) = %.0f ms" % (a.dataset, len(ts), fps, step, step / fps * 1000))
    tv, wv = visual_rate(a.dataset, a.calib, step, max_frames=a.max_frames)
    print("visual angular speed: %d samples, median %.2f rad/s, 95th percentile %.2f rad/s" % (len(wv), np.median(wv), np.percentile(wv, 95)))
    if a.selftest:
        rng = np.random.default_rng(1)
        true_offset = 0.053
        tg = np.arange(tv[0] - 1, tv[-1] + 1, 1.0 / 40.0)               # 40 Hz like the Cube
        tg += rng.uniform(-0.004, 0.004, len(tg))
        tg.sort()
        wg = np.interp(tg - true_offset, tv, wv) * 1.07 + rng.normal(0, 0.02, len(tg))   # IMU lags the camera by 53 ms
        bias = np.zeros(3)
        print("SELF-TEST: fake gyro is the visual rate delayed by %.0f ms, scaled 1.07, plus noise" % (true_offset * 1000))
    else:
        tg, wg, bias = gyro_rate(a.dataset, a.imu)
        print("gyro %s: %d samples, bias removed %s rad/s" % (a.imu, len(wg), np.round(bias, 4)))
    lag, corr, scale, (t, v, g, lags, cc) = estimate_offset(tv, wv, tg, wg)
    print("\nRESULT  camera-IMU offset = %+.1f ms   (IMU time = camera time + offset)" % (lag * 1000))
    print("        correlation at the best lag %.3f | visual/gyro scale %.3f" % (corr, scale))
    # how sharp is the peak: width where the correlation stays above 90 % of the peak
    above = lags[cc >= 0.9 * cc.max()]
    print("        peak width (>=90%% of max): %.0f ms" % ((above.max() - above.min()) * 1000))
    # stability: first half vs second half
    mid = len(tv) // 2
    try:
        l1 = estimate_offset(tv[:mid], wv[:mid], tg, wg)[0]
        l2 = estimate_offset(tv[mid:], wv[mid:], tg, wg)[0]
        print("        first half %+.1f ms | second half %+.1f ms" % (l1 * 1000, l2 * 1000))
    except RuntimeError as e:
        print("        (halves not compared: %s)" % e)
    quality = "GOOD" if corr > 0.8 else ("OK" if corr > 0.6 else "WEAK - move the rig more (all axes, some quick shakes)")
    print("        match quality: %s" % quality)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 7))
        ax[0].plot(t - t[0], v, label="camera |w| (rad/s)")
        ax[0].plot(t - t[0], g, label="gyro |w| shifted by %+.0f ms" % (lag * 1000), alpha=0.8)
        ax[0].set_xlabel("s"); ax[0].legend(); ax[0].set_title("rotation speed: camera vs IMU (aligned)")
        ax[1].plot(lags * 1000, cc); ax[1].axvline(lag * 1000, color="r")
        ax[1].set_xlabel("lag ms"); ax[1].set_ylabel("correlation"); ax[1].set_title("offset = %+.1f ms" % (lag * 1000))
        fig.tight_layout(); fig.savefig(a.out, dpi=110)
        print("        plot: %s" % a.out)
    except Exception as e:
        print("        (no plot: %s)" % e)


if __name__ == "__main__":
    main()
