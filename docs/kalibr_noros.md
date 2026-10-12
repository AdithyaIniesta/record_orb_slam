# Kalibr without ROS (camera and camera-IMU calibration)

ETH-ASL Kalibr built with plain CMake on Ubuntu, reading **folders of images and a CSV IMU file** instead of a rosbag.
Based on upstream commit `1f60227` (`kalibr_noros/kalibr-noros.patch`, verified to apply cleanly to that commit).
Tested: Ubuntu 22.04 x86_64 (WSL2), Python 3.10. **Not run on a Jetson (ARM) yet** - calibrate on the PC.

## Install
```bash
sudo ./kalibr_noros/setup_kalibr_noros.sh        # builds into /opt/kalibr_noros (10-30 min, needs internet)
```
The launcher is `/opt/kalibr_noros/kalibr-run <tool> ...` (older installs have `run.sh` - the scripts accept both).

## Camera-IMU calibration
1. Print `inputs/aprilgrid_kalibr_6x4_50mm_A3.pdf` at 100 % (A3 portrait, "actual size"), mount it flat and rigid, **measure
   one tag's black square** and put it in `tagSize` of the yaml (50 mm if printed exactly).
2. Record with RigCapture (IMU on): board **fixed**, rig moving, 60 s, rotations about all axes **and** real sideways /
   up-down / forward-back accelerations, board close (0.6-1 m) and in view. A hand-held board breaks the calibration.
3. `./kalibr_noros/calibrate_imu_camera.sh RECORDING_DIR [cam0|cam1]` (WSL / Linux).
4. Read `results/<recording>_<cam>/*-camchain-imucam.yaml`:
   - `T_cam_imu` = IMU -> camera (4x4); its last column is the **lever arm** in the camera frame.
   - `timeshift_cam_imu` = seconds, `t_imu = t_cam + shift` (about 12 ms here: mostly the Cube's gyro filter).
   - Good: reprojection error below about 0.5 px, IMU residuals about 1 or below, gravity magnitude 9.81.

**What to trust** (this rig, Cube Orange+ at 100 Hz): rotation about 0.1-0.3 deg run to run, time shift about 1 ms,
lever arm only about +-4 cm (it needs hard accelerations to show up) - measure it with calipers/CAD instead.
Combined result for this mount: `kalibr_noros/results/imu_to_camera_pair.yaml` (+ picture `rig_imu_cameras_frames.png` there).

## Notes that cost time
- **Do not print the AprilGrid PDFs of other tools:** Kalibr uses the AprilTag-2 tag36h11 table, a 2-cell black border and
  a y-up board (tag 0 bottom-left). `tools/make_kalibr_aprilgrid.py` makes a matching board (`--page A3|A4`, portrait).
- Two-camera recordings: cam1 frames are stamped with cam0's time but captured a few ms later (`pairs.csv`);
  the script restamps them. Only one camera can be calibrated per run (the board is rarely in both views).
- IMU at 40 Hz needs `--pose-knots-per-second 40 --bias-knots-per-second 20` (the script sets this from `meta.json`);
  Kalibr's default (100 knots/s) fits the IMU exactly and the extrinsics become meaningless.
- Kalibr writes its outputs next to the dataset name - copy them out before re-running.
- `--timeoffset-padding 0.08` is set because the first time-shift guess can be off by more than the default 30 ms.

## Camera calibration only (`kalibr_calibrate_cameras`)
```bash
/opt/kalibr_noros/kalibr-run kalibr_calibrate_cameras --bag DATASET --topics cam0 cam1 \
    --models pinhole-radtan pinhole-radtan --target inputs/aprilgrid_kalibr_6x4_50mm_A3.yaml --dont-show-report
```
`DATASET/cam0/<timestamp_ns>.png` (or a RigCapture recording: `DATASET/mav0/cam0/data/`); left/right frames of one
instant must share a file name. `--bag-freq 4` thins frames to about 4 Hz. `tools/synth_stereo.py` +
`tools/compare_to_truth.py` check the pipeline on rendered images with a known answer (all errors under 0.25 px / 0.1 mm).
Calibrate lenses from board views at several distances with strong tilts; views taken from a few similar poses cannot
separate focal length from distance (a test on our recordings gave a 30 % wrong focal length).

## What the patch changes
- build: `boost_python` follows the Python version; `boost/predef` instead of the removed `detail/endian.hpp`; catkin used as a plain build system.
- readers: `FolderImageDatasetReader` (images, EuRoC `mav0/cam0/data` or `cam0/`), `FolderImuDatasetReader` (`mav0/imu0/data.csv`).
- `kalibr_calibrate_cameras` / `kalibr_calibrate_imu_camera`: `--bag` may be a directory; no ROS imports;
  new options `--pose-knots-per-second`, `--bias-knots-per-second` (defaults unchanged).
- `PlotCollection.py`: `wx` GUI optional (headless); `MulticamGraph.py`: edge attribute newer python-igraph needs.
