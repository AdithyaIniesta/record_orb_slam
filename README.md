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
cd record_orb_slam
build_gui.bat
```
The result is `dist\RigCapture.exe` (one file, about 17 MB). Copy it anywhere and run it.

**First start on a new PC**
1. Set **Jetson IP** (and **SSH user**) in the top bar, then **Apply**.
2. Press **Set up Jetson login (once)**: a console opens - type the Jetson's password there when asked. (The app never
   sees the password; it only installs this PC's ssh key on the Jetson.) Press **Apply** again afterwards;
   **SSH login** in the Status box turns **OK**.
3. The app then starts the capture program on the Jetson by itself and the video appears. If it does not, check the
   firewall rule above and press **Start / restart capture on Jetson**.

## Recording format (on the Jetson, `~/recordings/stereo_NNNN/`)
EuRoC `mav0` layout, 8-bit gray PNG, timestamps = CLOCK_MONOTONIC ns of the V4L2 frame:
```
meta.json   times.txt
mav0/cam0/data/<timestamp_ns>.png      mav0/cam0/data.csv
mav0/cam1/...  pairs.csv                (only in "Both cameras" mode)
mav0/imu0/                              (reserved: IMU, not recorded yet)
```
Works directly with ORB-SLAM3's `mono_euroc` (`times.txt` is its timestamp file).

## Files
- `rig_capture.py` - the app (tkinter + Pillow). Video: H.264/RTP via GStreamer. Control: UDP 5010. Setup/copy: ssh.
- `jetson/stereo_capture.py` - the Jetson-side capture program (bundled into the exe, uploaded by the app).
- `build_gui.bat`, `requirements.txt`, `rig_capture.ico` - build the single-file exe with PyInstaller.
  `build_gui.bat` also refreshes `jetson/stereo_capture.py` from `..\jetson-tracking-perception\scripts\stereo_capture\`
  when that folder exists next to this one; otherwise it uses the copy in this repo.

## Plan this is built for
Mono boresight recording -> add an IMU on the same mount -> camera-IMU calibration -> visual-inertial odometry,
with an ablation with / without the IMU on the same recordings.
