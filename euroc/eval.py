#!/usr/bin/env python3
"""Evaluate ORB-SLAM3 EuRoC runs against Vicon ground truth.

Scans results/<seq>/<mode>/f_<seq>_<mode>.txt and writes a Markdown summary.
ATE RMSE: Sim3 alignment for mono/mono_inertial (scale free), SE3 for stereo variants.
Also reports tracked share (output rows / timestamp rows), ms per tracked frame
(from log mean), and reset count from run.log.

Depends only on numpy. Does not import ORB-SLAM3's evaluate_ate_scale.py so the
alignment math is explicit and reviewable.
"""
from __future__ import annotations
import argparse, csv, math, re, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
ORB_TS = Path("/opt/orbslam3/Examples/Monocular/EuRoC_TimeStamps")

SEQ_FOLDER = {"V1_01": "V1_01_easy", "V1_02": "V1_02_medium", "V1_03": "V1_03_difficult"}
SEQ_TS = {"V1_01": "V101.txt", "V1_02": "V102.txt", "V1_03": "V103.txt"}
MODES = ["mono", "mono_inertial", "stereo", "stereo_inertial"]
SCALE_MODES = {"mono"}  # mono-inertial resolves scale via IMU

# ---- IO ------------------------------------------------------------------

def load_tum(path: Path) -> np.ndarray:
    """Load TUM-format trajectory: t tx ty tz qx qy qz qw. Returns Nx8."""
    rows = []
    with path.open() as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 8:
                continue
            rows.append([float(x) for x in parts[:8]])
    arr = np.asarray(rows, dtype=np.float64)
    # ORB-SLAM3's SaveTrajectoryEuRoC writes timestamps in nanoseconds for
    # stereo / mono / stereo-inertial, but seconds for mono-inertial. Normalize
    # to seconds so associate() matches the Vicon ground truth.
    if len(arr) and arr[0, 0] > 1e10:
        arr[:, 0] *= 1e-9
    return arr


def load_asl_gt(csv_path: Path) -> np.ndarray:
    """Convert ASL state_groundtruth CSV to TUM: t[s] x y z qx qy qz qw."""
    out = []
    with csv_path.open() as f:
        rd = csv.reader(f)
        for row in rd:
            if not row or row[0].startswith("#"):
                continue
            t = float(row[0]) * 1e-9
            x, y, z = float(row[1]), float(row[2]), float(row[3])
            qw, qx, qy, qz = float(row[4]), float(row[5]), float(row[6]), float(row[7])
            out.append([t, x, y, z, qx, qy, qz, qw])
    return np.asarray(out, dtype=np.float64)


# ---- Alignment -----------------------------------------------------------

def associate(est: np.ndarray, gt: np.ndarray, max_dt: float = 0.02):
    """Nearest-neighbour match est timestamps to gt timestamps within max_dt."""
    gt_t = gt[:, 0]
    pairs = []
    for i, t in enumerate(est[:, 0]):
        j = int(np.searchsorted(gt_t, t))
        cand = []
        if j < len(gt_t): cand.append(j)
        if j > 0: cand.append(j - 1)
        if not cand:
            continue
        k = min(cand, key=lambda kk: abs(gt_t[kk] - t))
        if abs(gt_t[k] - t) <= max_dt:
            pairs.append((i, k))
    return pairs


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    """Umeyama alignment. src, dst: Nx3. Returns (R, t, s) so s*R@src+t ~ dst."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    S = src - mu_s
    D = dst - mu_d
    H = S.T @ D / len(src)
    U, s, Vt = np.linalg.svd(H)
    d = np.linalg.det(U @ Vt)
    W = np.diag([1.0, 1.0, 1.0 if d > 0 else -1.0])
    R = Vt.T @ W @ U.T
    if with_scale:
        var_s = (S * S).sum() / len(src)
        scale = (s @ np.diag(W).reshape(-1)) / max(var_s, 1e-12)
    else:
        scale = 1.0
    t = mu_d - scale * (R @ mu_s)
    return R, t, scale


def ate_rmse(est: np.ndarray, gt: np.ndarray, scale: bool) -> dict:
    pairs = associate(est, gt)
    if len(pairs) < 10:
        return {"matched": len(pairs), "rmse_m": None, "scale": None}
    E = est[[i for i, _ in pairs], 1:4]
    G = gt[[j for _, j in pairs], 1:4]
    R, t, s = umeyama(E, G, with_scale=scale)
    aligned = (s * (R @ E.T)).T + t
    err = np.linalg.norm(aligned - G, axis=1)
    return {
        "matched": len(pairs),
        "rmse_m": float(np.sqrt((err * err).mean())),
        "scale": float(s),
    }


# ---- Log parsing ---------------------------------------------------------

TIME_RE = re.compile(r"mean tracking time[:=]\s*([\d.]+)\s*ms", re.IGNORECASE)
TIME_RE2 = re.compile(r"median tracking time[:=]\s*([\d.]+)", re.IGNORECASE)
RESET_RE = re.compile(r"(RESET|Reseting|Reset active map|SYSTEM RESET)", re.IGNORECASE)


def parse_log(path: Path) -> dict:
    if not path.exists():
        return {"mean_ms": None, "resets": None}
    txt = path.read_text(errors="ignore")
    mean_ms = None
    m = TIME_RE.search(txt) or TIME_RE2.search(txt)
    if m:
        mean_ms = float(m.group(1))
    resets = len(RESET_RE.findall(txt))
    return {"mean_ms": mean_ms, "resets": resets}


# ---- Main ---------------------------------------------------------------

def count_lines(path: Path) -> int:
    n = 0
    with path.open() as f:
        for line in f:
            if line.strip() and not line.startswith("#"):
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=None, help="EuRoC data root (default: $EUROC_DATA or ~/euroc_data)")
    ap.add_argument("--seqs", nargs="*", default=["V1_01", "V1_02"])
    ap.add_argument("--out", default=str(HERE / "RESULTS.md"))
    args = ap.parse_args()

    import os
    data_root = Path(args.data or os.environ.get("EUROC_DATA") or (Path.home() / "euroc_data"))

    rows = []
    for seq in args.seqs:
        gt_csv = data_root / SEQ_FOLDER[seq] / "mav0" / "state_groundtruth_estimate0" / "data.csv"
        gt = load_asl_gt(gt_csv) if gt_csv.exists() else None
        ts_file = ORB_TS / SEQ_TS[seq]
        n_frames = count_lines(ts_file) if ts_file.exists() else None

        for mode in MODES:
            d = RESULTS / seq / mode
            traj = d / f"f_{seq}_{mode}.txt"
            log = d / "run.log"
            row = {"seq": seq, "mode": mode, "status": "missing"}
            if traj.exists():
                est = load_tum(traj)
                row["tracked"] = len(est)
                row["share"] = len(est) / n_frames if n_frames else None
                if gt is not None:
                    r = ate_rmse(est, gt, scale=(mode in SCALE_MODES))
                    row.update(r)
                row.update(parse_log(log))
                row["status"] = "ok"
            else:
                row.update(parse_log(log))
            rows.append(row)

    lines = ["# EuRoC comparison", "",
             "| seq | mode | ATE RMSE (m) | scale | tracked / total | ms/frame | resets |",
             "|-----|------|--------------|-------|-----------------|----------|--------|"]
    for r in rows:
        if r["status"] != "ok":
            lines.append(f"| {r['seq']} | {r['mode']} | — | — | — | — | — |  (no trajectory)")
            continue
        rmse = f"{r['rmse_m']:.4f}" if r.get("rmse_m") is not None else "—"
        scl = f"{r['scale']:.4f}" if r.get("scale") is not None and r["mode"] in SCALE_MODES else "1.0000" if r["mode"] not in SCALE_MODES else "—"
        shr = f"{r['tracked']}/{r.get('share') and int(round(r['tracked']/r['share']))}" if r.get("share") else f"{r['tracked']}/?"
        ms = f"{r['mean_ms']:.2f}" if r.get("mean_ms") is not None else "—"
        rst = str(r["resets"]) if r.get("resets") is not None else "—"
        lines.append(f"| {r['seq']} | {r['mode']} | {rmse} | {scl} | {shr} | {ms} | {rst} |")

    out = Path(args.out)
    out.write_text("\n".join(lines) + "\n")
    print(f"wrote {out}")
    print("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
