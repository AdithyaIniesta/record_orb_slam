# EuRoC comparison (V1_01 / V1_02 / V1_03)

Four ORB-SLAM3 modes on the Vicon Room 1 sequences:
mono, mono-inertial, stereo, stereo-inertial. Metrics: ATE RMSE vs Vicon
(Sim3 for mono, SE3 otherwise), tracked share, resets, ms/frame.

Everything runs in **WSL Ubuntu-22.04**. The host (Windows) only edits scripts.

## Prereqs (already in place)
- `/opt/orbslam3` with `build.sh` run; binaries `mono_euroc`, `mono_inertial_euroc`,
  `stereo_euroc`, `stereo_inertial_euroc` present.
- EuRoC zips at `C:\Users\admin\Downloads\vicon_room1\vicon_room1\V1_0{1,2,3}_*\*.zip`.

## Steps
```bash
# From inside WSL
cd /mnt/c/Users/admin/Documents/record_orb_slam/euroc

# 1. Unpack to WSL-native storage (~/euroc_data). Fast image I/O.
bash unpack.sh

# 2. Run everything on V1_01 and V1_02. ~5–15 min per trial, 8 trials.
bash run_all.sh            # defaults to V1_01 V1_02
# or: bash run_all.sh V1_01
# or: bash run_one.sh mono_inertial V1_02

# 3. Score against Vicon ground truth. Writes RESULTS.md.
python3 eval.py --seqs V1_01 V1_02
```

## Where things land
```
results/<seq>/<mode>/
  f_<seq>_<mode>.txt     # per-frame trajectory, TUM format (seconds)
  kf_<seq>_<mode>.txt    # keyframe trajectory
  run.log                # stdout+stderr of the trial
RESULTS.md               # ATE table built by eval.py
```

ORB-SLAM3 writes `f_*.txt` and `kf_*.txt` **only on a clean shutdown**. If a trial
crashes, the log remains but there is no trajectory to score.

## Notes carried from prior sessions
- Viewer wants X11 + software GL in WSL; `run_one.sh` exports
  `LIBGL_ALWAYS_SOFTWARE=1` and `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe` so Pangolin
  works whether the YAML has `Viewer.on: 1` or not.
- OpenCV settings numbers must be reals (`100.0`, not `100`) — the shipped
  `EuRoC.yaml` already is.
- `f_*.txt` from mono-inertial is relative to the first keyframe, so `eval.py`
  Sim3-aligns mono and SE3-aligns the three IMU/stereo cases, which absorbs the
  frame choice.
- Trajectories exist only after clean shutdown — don't Ctrl+C mid-run unless you
  want just `run.log`.

## V1_03
Hard sequence; expect frequent resets in mono. Add it with
`bash run_all.sh V1_01 V1_02 V1_03` once V1_01/02 look reasonable.
