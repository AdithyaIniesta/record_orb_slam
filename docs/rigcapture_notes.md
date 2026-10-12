# record_orb_slam

**Rig Capture** - a Windows recording app for a Jetson camera rig, in the same look and feel as the CvTracker
Ground Station (`jvp-ui`): themes (Dark / Light / Sunlight), connection bar, control column on the left, video on the
right, log at the bottom. It records mono (boresight camera) or stereo datasets in the EuRoC `mav0` layout, ready for
ORB-SLAM3 and, later, IMU / visual-inertial work. **Double-click `RigCapture.exe`** - that is all.

## What it does
- Live preview of the **boresight (6 mm) camera** (mono) or both cameras (Cameras box, then **Apply**).
- **START RECORDING / STOP RECORDING** with the mouse. "Start after 3 s" gives you time to pick the rig up.
- **Save at 30 / 60 frames/s**: 60 keeps every camera frame (better for fast motion, about 21 MB/s of disk; tested on
  an Orin NX with 0 drops).
- Exposure and gain sliders (applied on the Jetson while it runs). Focus is judged by eye - no focus meter.
- Status: link, ssh, fps, brightness, frames saved, rate, disk free; **Recordings on the Jetson** list with
  **Copy selected to this PC** (to `Documents\RigCapture\recordings` by default, changeable).
- Starts and updates the capture program on the Jetson by itself: on launch it checks whether the right version is
  running there; if not, it uploads the bundled `jetson/stereo_capture.py` over ssh and starts it. It never restarts
  anything while a recording is running.

## Installing on another Windows PC / laptop

**That PC needs** (same as for jvp-ui):
1. **GStreamer** (MSVC 64-bit runtime) with `gst-launch-1.0.exe` on PATH - it decodes the preview video.
   (`ffmpeg.exe` next to the app is the fallback decoder.)
2. The Windows **OpenSSH Client** (Settings > Apps > Optional features; present by default on Windows 10/11).
3. The same network as the Jetson. Allow `gst-launch-1.0.exe` through the Windows firewall for UDP (Private network).

**Option A - copy the exe (simplest, no Python needed):** copy `RigCapture.exe` from the PC where it was built
(USB stick, network share) into any folder and double-click it. It writes `rig_capture_config.json` next to itself.

**Option B - build it from this repo** (needs Python 3.9+ and internet once):
```bat
git clone git@github.com:AdithyaIniesta/record_orb_slam.git
cd record_orb_slam\rigcapture
build_gui.bat
```
The result is `rigcapture\dist\RigCapture.exe` (one file, about 17 MB). Copy it anywhere and run it.

**First start on a new PC**
1. Set **Jetson IP** (and **SSH user**) in the top bar, then **Apply**.
2. Press **Set up Jetson login (once)**: a console opens - type the Jetson's password there when asked. (The app never
   sees the password; it only installs this PC's ssh key on the Jetson.) Press **Apply** again afterwards;
   **SSH login** in the Status box turns **OK**.
3. The app then starts the capture program on the Jetson by itself and the video appears. If it does not, check the
   firewall rule above and press **Start / restart capture on Jetson**.

## IMU (Cube Orange on the Jetson's USB)
Plug the Cube Orange into the Jetson with a USB cable. The capture program on the Jetson reads it over MAVLink
(Python standard library only, nothing to install on the Jetson) and, while you record, writes it into the same
dataset on the same clock as the camera frames. Tick / untick **Record the IMU** in the Recording panel; the Status box
shows the IMU state and rate, and "IMU rows" counts the samples while recording.

**One-time permission on the Jetson** (the status shows `no access (dialout)` until this is done):
```bash
ssh orinnx1@192.168.0.40
sudo usermod -aG dialout orinnx1      # asks for the Jetson password; then close that ssh session
```
Then press **Start / restart capture on Jetson** in the app (a fresh login is needed for the new group to apply).

What is recorded (`mav0/`): `imu0` = Cube IMU1, `imu1` = IMU2, `imu2` = IMU3, each with `data.csv` (EuRoC IMU format:
timestamp ns, gyro rad/s, accel m/s^2), `data_fc.csv` (the Cube's own time and the raw arrival time) and `sensor.yaml`;
`cube/attitude.csv` (the Cube's EKF attitude) and `cube/timesync.csv` (clock-offset estimates, Cube minus Jetson).
Timestamps are the Cube's own sample time moved onto the Jetson's CLOCK_MONOTONIC using the clock offset of the
lowest-latency messages (the TIMESYNC round trip is asymmetric on this Cube - it answers only on its 50 Hz loop tick -
so it is logged for diagnostics but not used), so they are steady, strictly increasing and never later than the
arrival time (measured: about 1.4 ms before arrival on average). Samples come in the Cube's 50 Hz loop pattern
(mostly 20 ms apart, occasional 40-80 ms gaps, and a few duplicates 0.05 ms apart that you may drop when processing). Frame: FRD (x forward, y right, z down); accelerometer = specific force.

**IMU rate (measured on this Cube, ArduPilot 4.7.0):** the Cube streams at most about half its main-loop rate
(`SCHED_LOOP_RATE`): loop 50 Hz -> about 40 Hz (irregular), loop 200 Hz -> a steady **100 Hz**. This Cube is set to 200.
The app asks for **100 Hz** and steps down by itself (100 / 50 / 40 / 25 Hz) if the Cube refuses, so it also works with
a loop of 50. Each value is the Cube's latest sample at send time in 16-bit integers (1 mg, 1 mrad/s), not every raw
sample, and it is the Cube's low-pass filtered IMU (about 12 ms of delay, which ends up in the camera-IMU time shift).
That is enough for the camera-IMU calibration (see `../kalibr_noros`); for flight-grade visual-inertial work a separate
IMU on the Jetson, or raw logging, would still be better.
The program only asks the Cube for telemetry rates (reset at Cube reboot, asked back to default when the capture
program stops), never arms, never changes parameters, and does nothing about rates while the vehicle is armed.

## Recording format (on the Jetson, `~/recordings/stereo_NNNN/`)
EuRoC `mav0` layout, 8-bit gray PNG, timestamps = CLOCK_MONOTONIC ns of the V4L2 frame:
```
meta.json   times.txt
mav0/cam0/data/<timestamp_ns>.png      mav0/cam0/data.csv
mav0/cam1/...  pairs.csv                (only in "Both cameras" mode)
mav0/imu0/ imu1/ imu2/  cube/            (Cube IMUs + attitude + timesync, see above)
```
Works directly with ORB-SLAM3's `mono_euroc` (`times.txt` is its timestamp file).

## Files
(all in `rigcapture/`)
- `rig_capture.py` - the app (tkinter + Pillow). Video: H.264/RTP via GStreamer. Control: UDP 5010. Setup/copy: ssh.
- `jetson/stereo_capture.py` - the Jetson-side capture program (bundled into the exe, uploaded by the app). This copy is
  the source of truth.
- `build_gui.bat`, `requirements.txt`, `rig_capture.ico` - build the single-file exe with PyInstaller.
- `tools/cam_imu_sync.py` - camera-IMU time offset from a recording (no board needed);
  `tools/replay_orbslam.py` - video of an ORB-SLAM3 run (camera image + 3D path) without any 3D viewer.
- `tests/` - self-tests of the IMU recorder against a fake Cube (needs `pymavlink`, run on Linux/WSL):
  `python3 tests/test_imu_recorder.py jetson/stereo_capture.py`, `python3 tests/test_imu_stepdown.py jetson/stereo_capture.py`.

## Plan this is built for
Mono boresight recording -> add an IMU on the same mount -> camera-IMU calibration -> visual-inertial odometry,
with an ablation with / without the IMU on the same recordings.
