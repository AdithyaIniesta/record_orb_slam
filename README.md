# record_orb_slam

Record camera + IMU data on a Jetson rig, calibrate camera-IMU, run ORB-SLAM3.

| folder | what | where it runs |
|---|---|---|
| `rigcapture/` | recording app (preview, record, copy) | Windows PC / laptop (+ capture program on the Jetson, uploaded by the app) |
| `kalibr_noros/` | Kalibr without ROS: camera-IMU calibration, printable board, results | WSL / Ubuntu |
| `orbslam3/` | ORB-SLAM3 build + run scripts and settings | WSL / Ubuntu (Jetson: next) |

## 1. RigCapture (Windows)
Needs: **GStreamer** (`gst-launch-1.0` on PATH), the Windows **OpenSSH client** (built in), same network as the Jetson.
```bat
cd rigcapture
build_gui.bat          :: once, needs Python 3.9+ and internet -> dist\RigCapture.exe  (or just copy the exe)
```
1. Double-click `RigCapture.exe`. Enter the Jetson **IP** and **SSH user**, press **Apply**.
2. **Set up Jetson login (once)**: type the Jetson password in the console that opens, then **Apply** again.
3. Jetson, once: Cube Orange on USB and `sudo usermod -aG dialout <user>`.
4. Pick the camera(s), **Save at 30** (two cameras) or 60 (one camera), press **START RECORDING**, **STOP**.
5. **Copy selected to this PC** -> `recordings\stereo_NNNN` (EuRoC layout, works with ORB-SLAM3 and Kalibr).

## 2. Kalibr without ROS (WSL / Ubuntu 22.04)
```bash
sudo ./kalibr_noros/setup_kalibr_noros.sh                       # once, 10-30 min
./kalibr_noros/calibrate_imu_camera.sh RECORDING_DIR cam0       # cam0 = 6 mm boresight, cam1 = 8 mm depression
```
Print `kalibr_noros/inputs/aprilgrid_kalibr_6x4_50mm_A3.pdf` at 100 %, fix it flat and still, measure a tag and set
`tagSize`. Record 60 s moving only the rig. Result: `kalibr_noros/results/<recording>_<cam>/` (`T_cam_imu`, time shift).
Details and what to trust: `docs/kalibr_noros.md`.

On the Jetson (JetPack 5, uses the CUDA OpenCV in `/opt/opencv-4.10.0-cuda`): `./kalibr_noros/setup_kalibr_jetson.sh` (no root; it prints
the one `sudo apt install` line it needs), then the same `calibrate_imu_camera.sh`. Not yet tested on a device.

## 3. ORB-SLAM3 (WSL / Ubuntu 22.04)
```bash
sudo ./orbslam3/build_orbslam3.sh                               # once, clones upstream, 20-40 min
./orbslam3/run_mono.sh RECORDING_DIR orbslam3/settings/orbslam_mono_boresight.yaml
python3 rigcapture/tools/replay_orbslam.py RECORDING_DIR RECORDING_DIR/orbslam_*/f_*.txt --out replay.mp4   # video of the run (needs numpy, matplotlib, opencv)
```
`run_mono.sh` prints how many frames got a pose. Monocular without IMU: arbitrary scale, needs sideways motion.

### Live SLAM on the Jetson (JetPack 5)
```bash
sudo apt install -y libssl-dev libboost-serialization-dev libglew-dev libepoxy-dev     # once, the only sudo step
./orbslam3/build_orbslam3_jetson.sh                                                    # once, no root, 20-40 min, installs in ~/orbslam
```
Then press **START LIVE SLAM** in RigCapture: a 3D window shows the camera path. Move the rig slowly **sideways** until it says
TRACKING. (No GUI: `./orbslam3/run_live.sh`.) Real-time settings: 640x360, 1500 features, about 27-30 fps on an Orin NX.
Problems met and their fixes: `docs/orbslam3_jetson.md`.

## Status
- Verified: RigCapture (Windows 11, exe builds from scratch, IMU 100 Hz, live SLAM button + 3D view), Kalibr camera-IMU on real recordings,
  ORB-SLAM3 in WSL and live on a Jetson Orin NX (JetPack 5.1.5).
- Not done yet: visual-inertial runs (Cube IMU into ORB-SLAM3), exact lever arm (measure with calipers), install on a second Jetson.
- More: `docs/rigcapture_notes.md` (IMU format, limits), `docs/kalibr_noros.md`, `docs/orbslam3_jetson.md`.
