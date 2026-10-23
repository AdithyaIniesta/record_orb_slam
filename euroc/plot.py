#!/usr/bin/env python3
"""Plot estimated trajectories against Vicon ground truth, per sequence.

Writes plots/<seq>_xy.png (top-down) and plots/<seq>_err.png (|err| over time)
with one subplot per mode. Alignment follows eval.py: Sim3 for mono, SE3 otherwise.
"""
from __future__ import annotations
import os, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import eval as E

MODES = ["mono", "mono_inertial", "stereo", "stereo_inertial"]
TITLES = {"mono": "mono", "mono_inertial": "mono-inertial",
          "stereo": "stereo", "stereo_inertial": "stereo-inertial"}


def align(est_xyz, gt_xyz, with_scale):
    R, t, s = E.umeyama(est_xyz, gt_xyz, with_scale=with_scale)
    return (s * (R @ est_xyz.T)).T + t


def process_seq(seq, data_root, out_dir):
    gt_csv = data_root / E.SEQ_FOLDER[seq] / "mav0" / "state_groundtruth_estimate0" / "data.csv"
    gt = E.load_asl_gt(gt_csv)

    fig_xy, axes_xy = plt.subplots(2, 2, figsize=(10, 10))
    fig_err, axes_err = plt.subplots(2, 2, figsize=(12, 7), sharey=True)
    fig_xy.suptitle(f"{seq}  —  top-down (XY) after alignment", fontsize=13)
    fig_err.suptitle(f"{seq}  —  |position error| over time (m)", fontsize=13)

    for ax, mode in zip(axes_xy.flat, MODES):
        traj = HERE / "results" / seq / mode / f"f_{seq}_{mode}.txt"
        if not traj.exists():
            ax.set_title(f"{TITLES[mode]}  (missing)")
            ax.axis("off"); continue
        est = E.load_tum(traj)
        pairs = E.associate(est, gt)
        if len(pairs) < 10:
            ax.set_title(f"{TITLES[mode]}  (unmatched)")
            ax.axis("off"); continue
        Ei = np.array([p[0] for p in pairs]); Gi = np.array([p[1] for p in pairs])
        E_xyz = est[Ei, 1:4]; G_xyz = gt[Gi, 1:4]
        with_scale = (mode in E.SCALE_MODES)
        A_xyz = align(E_xyz, G_xyz, with_scale)
        err = np.linalg.norm(A_xyz - G_xyz, axis=1)
        rmse = float(np.sqrt((err*err).mean()))

        ax.plot(G_xyz[:,0], G_xyz[:,1], color="#333", lw=1.4, label="Vicon GT")
        ax.plot(A_xyz[:,0], A_xyz[:,1], color="#d62728", lw=1.0, alpha=0.9, label="ORB-SLAM3")
        ax.scatter(G_xyz[0,0], G_xyz[0,1], color="green", s=25, zorder=3, label="start")
        ax.set_title(f"{TITLES[mode]}  RMSE={rmse:.3f} m")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="best")

    for ax, mode in zip(axes_err.flat, MODES):
        traj = HERE / "results" / seq / mode / f"f_{seq}_{mode}.txt"
        if not traj.exists():
            ax.set_title(f"{TITLES[mode]}  (missing)"); ax.axis("off"); continue
        est = E.load_tum(traj)
        pairs = E.associate(est, gt)
        if len(pairs) < 10:
            ax.set_title(f"{TITLES[mode]}  (unmatched)"); ax.axis("off"); continue
        Ei = np.array([p[0] for p in pairs]); Gi = np.array([p[1] for p in pairs])
        E_xyz = est[Ei, 1:4]; G_xyz = gt[Gi, 1:4]
        t = est[Ei, 0]; t = t - t[0]
        A_xyz = align(E_xyz, G_xyz, with_scale=(mode in E.SCALE_MODES))
        err = np.linalg.norm(A_xyz - G_xyz, axis=1)
        ax.plot(t, err, color="#1f77b4", lw=0.9)
        ax.axhline(np.sqrt((err*err).mean()), color="#d62728", ls="--", lw=0.8,
                   label=f"RMSE {np.sqrt((err*err).mean()):.3f}")
        ax.set_title(TITLES[mode])
        ax.set_xlabel("t (s)"); ax.set_ylabel("|err| (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    out_dir.mkdir(parents=True, exist_ok=True)
    p1 = out_dir / f"{seq}_xy.png"; p2 = out_dir / f"{seq}_err.png"
    fig_xy.tight_layout(); fig_xy.savefig(p1, dpi=130); plt.close(fig_xy)
    fig_err.tight_layout(); fig_err.savefig(p2, dpi=130); plt.close(fig_err)
    print(f"  {p1}\n  {p2}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.environ.get("EUROC_DATA") or str(Path.home() / "euroc_data"))
    ap.add_argument("--seqs", nargs="*", default=["V1_01", "V1_02"])
    ap.add_argument("--out", default=str(HERE / "plots"))
    args = ap.parse_args()
    data_root = Path(args.data)
    out_dir = Path(args.out)
    for seq in args.seqs:
        print(f"[{seq}]")
        process_seq(seq, data_root, out_dir)


if __name__ == "__main__":
    main()
