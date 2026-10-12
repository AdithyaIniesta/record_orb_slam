"""Render a synthetic stereo AprilGrid dataset with known ground truth.

Usage: synth.py OUTDIR [nframes]
Writes OUTDIR/cam0/<ns>.png, OUTDIR/cam1/<ns>.png and OUTDIR/truth.json
"""
import json, os, sys
import cv2
import numpy as np

out = sys.argv[1]
nframes = int(sys.argv[2]) if len(sys.argv) > 2 else 60
page = cv2.imread("/opt/kalibr_noros/test/page.png", 0)
PPM = 150 / 0.0254                     # page pixels per metre
W, H = 1280, 720
rng = np.random.default_rng(7)

cams = {
    "cam0": dict(K=np.array([[800., 0, 640], [0, 800, 360], [0, 0, 1]]),
                 dist=np.array([-0.20, 0.05, 0.001, -0.001, 0.0])),
    "cam1": dict(K=np.array([[805., 0, 645], [0, 805, 355], [0, 0, 1]]),
                 dist=np.array([-0.18, 0.04, -0.0008, 0.0006, 0.0])),
}
# cam1 relative to cam0: X_c1 = R10 X_c0 + t10
R10 = cv2.Rodrigues(np.array([0.004, -0.012, 0.006]))[0]
t10 = np.array([-0.100, 0.002, 0.001])
C1_in_c0 = -R10.T @ t10                # cam1 centre in cam0 frame

u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
pix = np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2)
rays = {}
for name, c in cams.items():
    n = cv2.undistortPoints(pix, c["K"], c["dist"]).reshape(-1, 2)
    d = np.concatenate([n, np.ones((len(n), 1))], 1)
    rays[name] = d


def render(name, Rb, tb):
    d = rays[name]
    if name == "cam0":
        C, d0 = np.zeros(3), d
    else:
        C, d0 = C1_in_c0, d @ R10          # (R10^T d) as row vectors
    nrm = Rb[:, 2]
    s = (nrm @ (tb - C)) / (d0 @ nrm)
    X = C + s[:, None] * d0                # points in cam0 frame
    Xb = (X - tb) @ Rb                     # Rb^T (X - tb)
    px = Xb[:, 0] * PPM + page.shape[1] / 2
    py = -Xb[:, 1] * PPM + page.shape[0] / 2
    bad = s <= 0
    px[bad] = -1; py[bad] = -1
    img = cv2.remap(page, px.reshape(H, W).astype(np.float32), py.reshape(H, W).astype(np.float32),
                    cv2.INTER_AREA if False else cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=128)
    img = cv2.GaussianBlur(img, (3, 3), 0.7)
    noise = rng.normal(0, 2.0, img.shape)
    return np.clip(img.astype(np.float64) + noise, 0, 255).astype(np.uint8)


for n in cams:
    os.makedirs(os.path.join(out, n), exist_ok=True)
t0 = 1700000000 * 10**9
for i in range(nframes):
    ang = rng.uniform(-0.45, 0.45, 3)
    Rb = cv2.Rodrigues(ang)[0] @ np.diag([1.0, -1.0, -1.0])   # board faces the camera
    Rb = Rb if np.linalg.det(Rb) > 0 else -Rb
    z = rng.uniform(0.85, 1.5)
    tb = np.array([rng.uniform(-0.12, 0.12) * z, rng.uniform(-0.08, 0.08) * z, z]) + np.array([0.05, 0, 0])
    ts = t0 + i * 250_000_000
    for n in cams:
        cv2.imwrite(os.path.join(out, n, f"{ts}.png"), render(n, Rb, tb))

json.dump({k: {"K": v["K"].tolist(), "dist": v["dist"].tolist()} for k, v in cams.items()} |
          {"R10": R10.tolist(), "t10": t10.tolist(), "baseline_m": float(np.linalg.norm(t10))},
          open(os.path.join(out, "truth.json"), "w"), indent=1)
print("wrote", nframes, "frames to", out)
