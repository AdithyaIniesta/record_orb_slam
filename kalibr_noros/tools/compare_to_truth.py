"""Compare a Kalibr camchain.yaml with the synthetic ground truth written by synth_stereo.py.

usage: python3 compare_to_truth.py DATASET_DIR/truth.json DATASET_DIR-camchain.yaml
"""
import json, sys
import numpy as np
import yaml

truth = json.load(open(sys.argv[1]))
chain = yaml.safe_load(open(sys.argv[2]))

for cam in ("cam0", "cam1"):
    K = np.array(truth[cam]["K"])
    gt_int = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
    est = chain[cam]["intrinsics"]
    print(f"{cam} fx fy cx cy  truth {np.round(gt_int, 2)}  est {np.round(est, 2)}  max err {np.max(np.abs(np.array(est) - gt_int)):.3f} px")
    gt_d = truth[cam]["dist"][:4]
    ed = chain[cam]["distortion_coeffs"]
    print(f"{cam} k1 k2 p1 p2   truth {np.round(gt_d, 4)}  est {np.round(ed, 4)}")

T = np.array(chain["cam1"]["T_cn_cnm1"])
R10, t10 = np.array(truth["R10"]), np.array(truth["t10"])
cosang = (np.trace(R10.T @ T[:3, :3]) - 1) / 2
print(f"baseline  truth {np.linalg.norm(t10)*1000:.3f} mm   est {np.linalg.norm(T[:3, 3])*1000:.3f} mm")
print(f"t10 error {np.linalg.norm(T[:3, 3] - t10)*1000:.3f} mm   rotation error {np.degrees(np.arccos(np.clip(cosang, -1, 1))):.4f} deg")
