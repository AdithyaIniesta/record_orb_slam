#!/usr/bin/env python3
"""
replay_orbslam.py - make a video of an ORB-SLAM3 run without any 3D viewer window (no OpenGL needed).

    python3 replay_orbslam.py DATASET_DIR TRAJECTORY.txt --kf KEYFRAMES.txt --out replay.mp4 [--step 2]

DATASET_DIR     recording in the mav0 layout (mav0/cam0/data/<ns>.png + times.txt)
TRAJECTORY.txt  ORB-SLAM3 EuRoC output (SaveTrajectoryEuRoC): t_ns tx ty tz qx qy qz qw  (camera pose in the map)
--kf            optional keyframe trajectory, same format

Left: camera image with the tracking state. Right: the camera path in 3D, the current camera (frustum) and the keyframes.
Map units are arbitrary for monocular without IMU (the scale is unknown). Axes are equal, so the shape is right.
Needs numpy, matplotlib, opencv (no ffmpeg).
"""
import argparse
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def frustum(p, R, s):
    """Pyramid of a camera at p looking along its +z (ORB convention: x right, y down, z forward)."""
    w, h, d = 0.5 * s, 0.35 * s, 0.8 * s
    corners = np.array([[-w, -h, d], [w, -h, d], [w, h, d], [-w, h, d]])
    c = (R @ corners.T).T + p
    return c


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("trajectory")
    ap.add_argument("--kf")
    ap.add_argument("--inertial", metavar="CAMIMU.yaml",
                    help="the trajectory is from monocular-INERTIAL ORB-SLAM3 (IMU body poses in a gravity-aligned world, metres): "
                         "give the Kalibr camera-IMU result (T_imu_cam) to draw the camera, z up; f_ and kf_ files are put in one frame")
    ap.add_argument("--out", default="replay.mp4")
    ap.add_argument("--step", type=int, default=2, help="use every Nth frame (2 = 30 fps video from a 60 fps recording)")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1600)
    a = ap.parse_args()

    ts = np.array([int(l) for l in open(os.path.join(a.dataset, "times.txt")) if l.strip()], dtype=np.int64)
    tr = np.loadtxt(a.trajectory)
    tr = tr.reshape(-1, 8)
    tt = np.round(tr[:, 0]).astype(np.int64) if tr[0, 0] > 1e11 else np.round(tr[:, 0] * 1e9).astype(np.int64)
    P = tr[:, 1:4]
    kf = np.loadtxt(a.kf).reshape(-1, 8) if a.kf and os.path.exists(a.kf) else np.zeros((0, 8))
    Rall = np.array([quat_to_rot(q) for q in tr[:, 4:8]])
    Rkf = np.array([quat_to_rot(q) for q in kf[:, 4:8]]) if len(kf) else np.zeros((0, 3, 3))
    Pkf = kf[:, 1:4] if len(kf) else np.zeros((0, 3))
    if a.inertial:
        import yaml
        Tbc = np.array(yaml.safe_load(open(a.inertial))["T_imu_cam"], dtype=float)
        # ORB-SLAM3 writes the frame file (f_) relative to the first keyframe's IMU pose and the keyframe file (kf_) in the map frame:
        # same trajectory, two frames. Put f_ into the kf_ frame with a rigid fit on the shared time stamps.
        if len(kf):
            fi = {int(round(t)): i for i, t in enumerate(tr[:, 0])}
            pr = [(fi[int(round(t))], j) for j, t in enumerate(kf[:, 0]) if int(round(t)) in fi]
            if len(pr) >= 3:
                A = np.array([P[i] for i, _ in pr]); B = np.array([Pkf[j] for _, j in pr])
                ca, cb = A.mean(0), B.mean(0)
                U, S, Vt = np.linalg.svd((A - ca).T @ (B - cb)); Rf = Vt.T @ U.T
                if np.linalg.det(Rf) < 0:
                    Vt[-1] *= -1; Rf = Vt.T @ U.T
                P = (Rf @ (P - ca).T).T + cb
                Rall = np.array([Rf @ r for r in Rall])
                print("f_ put into the kf_ frame: residual %.3f m" % np.linalg.norm((Rf @ (A - ca).T).T + cb - B, axis=1).mean())
        # IMU body pose -> camera pose: Twc = Twb * Tbc
        P = np.array([r @ Tbc[:3, 3] + p for r, p in zip(Rall, P)]); Rall = np.array([r @ Tbc[:3, :3] for r in Rall])
        Pkf = np.array([r @ Tbc[:3, 3] + p for r, p in zip(Rkf, Pkf)]) if len(kf) else Pkf
    # monocular (no IMU): camera axes x right / y down / z forward, plot x, z, -y.  inertial: gravity-aligned world, z up, plot x, y, z.
    def to_plot(p):
        return np.array(p, dtype=float) if a.inertial else np.array([p[0], p[2], -p[1]])
    Pp = np.array([to_plot(p) for p in P])
    Kp = np.array([to_plot(p) for p in Pkf]) if len(kf) else np.zeros((0, 3))

    lo, hi = Pp.min(0), Pp.max(0)
    c = (lo + hi) / 2
    half = max((hi - lo).max() / 2, 1e-3) * 1.25
    cam_size = half * 0.18

    # frame -> pose index (nearest in time within 12 ms); -1 = no pose (initialising / lost)
    idx = np.full(len(ts), -1)
    j = np.searchsorted(tt, ts)
    for i, t in enumerate(ts):
        best, bd = -1, 12e6
        for k in (j[i] - 1, j[i]):
            if 0 <= k < len(tt) and abs(tt[k] - t) < bd:
                best, bd = k, abs(tt[k] - t)
        idx[i] = best

    W = a.width
    H = int(W * 0.44)
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    ax_img = fig.add_axes([0.01, 0.02, 0.48, 0.96])
    ax_img.axis("off")
    ax3 = fig.add_axes([0.50, 0.02, 0.49, 0.96], projection="3d")

    out = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    frames = list(range(0, len(ts), a.step))
    t0 = ts[0]
    trail_len = 10 ** 9
    for n, i in enumerate(frames):
        img = cv2.imread(os.path.join(a.dataset, "mav0", "cam0", "data", "%d.png" % ts[i]), 0)
        ax_img.clear(); ax_img.axis("off")
        if img is not None:
            ax_img.imshow(img, cmap="gray", vmin=0, vmax=255)
        k = idx[i]
        state = "TRACKING" if k >= 0 else "NO POSE (initialising / lost)"
        ax_img.text(0.02, 0.97, "t = %5.2f s   frame %d   %s" % ((ts[i] - t0) / 1e9, i, state), transform=ax_img.transAxes,
                    va="top", fontsize=12, color="#7CFC00" if k >= 0 else "#FF5555",
                    bbox=dict(facecolor="black", alpha=0.6, pad=3, edgecolor="none"))

        ax3.clear()
        ax3.set_xlim(c[0] - half, c[0] + half)
        ax3.set_ylim(c[1] - half, c[1] + half)
        ax3.set_zlim(c[2] - half, c[2] + half)
        ax3.set_box_aspect((1, 1, 1))
        if a.inertial:
            ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]"); ax3.set_zlabel("z up [m]")
            ax3.set_title("ORB-SLAM3 monocular-inertial camera path (metres)", fontsize=10)
        else:
            ax3.set_xlabel("x"); ax3.set_ylabel("z (forward)"); ax3.set_zlabel("-y (up)")
            ax3.set_title("ORB-SLAM3 camera path (map units, scale arbitrary)", fontsize=10)
        ax3.plot(Pp[:, 0], Pp[:, 1], Pp[:, 2], color="#bbbbbb", lw=0.8)
        if len(Kp):
            shown = Kp[kf[:, 0] <= tt[k]] if k >= 0 else Kp[:0]
            if len(shown):
                ax3.scatter(shown[:, 0], shown[:, 1], shown[:, 2], s=14, c="#1f77b4", depthshade=False)
        if k >= 0:
            ax3.plot(Pp[:k + 1, 0], Pp[:k + 1, 1], Pp[:k + 1, 2], color="#d62728", lw=1.8)
            R = Rall[k]
            p = P[k]
            cs = frustum(p, R, cam_size)
            cs = np.array([to_plot(q) for q in cs])
            pp = to_plot(p)
            for q in cs:
                ax3.plot([pp[0], q[0]], [pp[1], q[1]], [pp[2], q[2]], color="#2ca02c", lw=1.5)
            ring = np.vstack([cs, cs[:1]])
            ax3.plot(ring[:, 0], ring[:, 1], ring[:, 2], color="#2ca02c", lw=1.5)
        fig.canvas.draw()
        frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        if n % 50 == 0:
            print("  frame %d / %d" % (n, len(frames)), flush=True)
    out.release()
    print("wrote", a.out, "(%d frames, %.0f fps)" % (len(frames), a.fps))


if __name__ == "__main__":
    main()
