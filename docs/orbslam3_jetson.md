# ORB-SLAM3 on a Jetson (JetPack 5) - install, speed and problems log

Target: JetPack 5 (L4T R35.x, Ubuntu 20.04, Python 3.8, aarch64). First machine: Orin NX 16 GB, 8 cores, JetPack 5.1.5 (R35.6.4).
Everything here was done on that machine; each problem is listed with its fix so a new Jetson goes smoothly.

## Install (new Jetson, JetPack 5)
```bash
git clone git@github.com:AdithyaIniesta/record_orb_slam.git && cd record_orb_slam/orbslam3
sudo apt install -y libssl-dev libboost-serialization-dev libglew-dev libepoxy-dev     # the only sudo step
./build_orbslam3_jetson.sh                                                              # no root, 20-40 min
./run_mono.sh ~/recordings/stereo_NNNN settings/orbslam_mono_boresight.yaml            # headless; prints the tracked share
```
Everything lands under **`~/orbslam/`** (`ORB_HOME=...` to change): `src/` (pinned upstream + Pangolin sources), `orbslam3/` (patched
build), `local/` (Pangolin), `apps/` (our programs), `logs/` (all build logs). Nothing else in the home folder is touched.
If a package is missing the script stops and prints the exact `sudo apt install ...` line.
`VIEWER=1 ./run_mono.sh ...` shows the ORB-SLAM3 windows (needs a display; Jetsons are usually headless).

## Speed (Orin NX, monocular, `mono_bench` on a 60 fps recording of 917 frames, capture program running at the same time)
Per-frame `TrackMonocular` time; real time at 30 fps needs a mean well below 33 ms **and** few frames above it.

| features | processing size | mean ms | 95th ms | throughput fps | frames with a pose |
|---|---|---|---|---|---|
| 3000 (our PC setting) | 1280x720 | 62.6 | 126 | 16.0 | 94 % |
| 1500 | 1280x720 | 46.4 | 86 | 21.6 | 95 % |
| 1000 | 1280x720 | 40.4 | 66 | 24.8 | 93 % |
| 1500 | 960x540 | 35.5 | 55 | 28.2 | 95 % |
| 1000 | 960x540 | 30.6 | 61 | 32.7 | 94 % |
| 2000 | 640x360 | 31.1 | 68 | 32.2 | 95 % |
| 1500 | 640x360 | 28.7 | 79 | 34.9 | 94 % |
| 1000 | 640x360 | 22.3 | 41 | 44.8 | 95 % |

On the 27 s recording (`stereo_0010`) the default 3000 features at 1280x720 gave 84 ms mean (11.8 fps). **The PC settings are not real time on
the Jetson**; 640x360 with 1000-1500 features is. Set the processing size with `Camera.newWidth` / `Camera.newHeight` in the settings file.
Single frames still take 2-3x the mean (new keyframes), so a live program must always work on the **newest** frame and drop older ones.

## Problems met, in the order they appeared
| # | problem | symptom | fix / status |
|---|---|---|---|
| 1 | `sudo` needs a password | automated installs cannot run `apt` | one `apt install` line typed by the user; everything else runs as the normal user |
| 2 | JetPack lacks four dev packages | `fatal error: boost/serialization/serialization.hpp` (first error, in DBoW2); also missing `openssl/md5.h`, `GL/glew.h`, `epoxy/gl.h` | `libboost-serialization-dev libssl-dev libglew-dev libepoxy-dev`; the script checks all of them first |
| 3 | upstream clone is 1.6 GB | long clone, wasted disk | a shallow fetch only saves the history (`.git` 180 MB); the working tree is still 1.6 GB, 1.3 GB of it TUM-VI example data (`TUM_IMU`). The script keeps a bare shallow repo and exports with `git archive` without `TUM_IMU` (about 0.3 GB). (The "79 MB" first noted here was a reading taken mid-download.) |
| 4 | two OpenCV installs | JetPack 4.5.4 in `/usr/lib/aarch64-linux-gnu` and a CUDA 4.10 in `/opt/opencv-4.10.0-cuda`; CMake could pick either | the script pins `OpenCV_DIR` to the system 4.5.4 (ORB-SLAM3 needs >= 4.4) |
| 5 | WSL build had an undocumented edit | WSL `mono_euroc` showed a viewer (`true`), upstream and Jetson have `false`; the edit was in no patch | `viewer_env.patch`: all EuRoC examples are headless unless `ORB_VIEWER=1` |
| 6 | `curl` and `/usr/bin/time` are not installed | scripts using them fail | use `wget`, `git`, the shell `time` builtin |
| 7 | device clocks are wrong | `make` can think objects are up to date and compile nothing | the script builds from a fresh copy every time and checks that `Linking CXX shared library` appears |
| 8 | the capture program uses ~75 % of a core | slower builds and slower SLAM while it runs | measured with it running (realistic); stop it for a faster build |
| 9 | the upstream example prints no per-frame times and sleeps to the recording's timestamps | wall time = recording length, nothing to compare | `apps/mono_bench.cc` times every `TrackMonocular` call without pacing |
| 10 | `Camera.imageScale` in settings is ignored (new-format settings) | identical timings for scale 1.0 / 0.75 / 0.5 | the keys are `Camera.newWidth` / `Camera.newHeight`; the caller resizes with `System::GetImageScale()` |
| 11 | defaults are too heavy for real time | 3000 features at 1280x720: 12-16 fps | see the speed table: 640x360 with 1000-1500 features |
| 12 | my first run of a new program passed `0` as a frame limit | "0 frames" | `0` now means all frames |
| 13 | home folder cluttered with logs and source trees | many `orb_*.log`, `Pangolin`, `ORB_SLAM3_upstream`, `local` | everything under `~/orbslam/`, logs in `~/orbslam/logs` |
| 14 | our own programs could not find a Pangolin library | `mono_live: error while loading shared libraries: libpango_windowing.so` | plain RUNPATH is not used for dependencies of dependencies; `apps/CMakeLists.txt` sets an old-style RPATH (`--disable-new-dtags`), no `LD_LIBRARY_PATH` needed |
| 15 | DNS is empty after a reboot | `Could not resolve host: github.com`, although ping to 8.8.8.8 works and NetworkManager holds 8.8.8.8 / 1.1.1.1 | boot-order race between NetworkManager and resolvconf (`/etc/resolv.conf` empty). One-time fix with sudo: `sudo resolvconf -u` (or `sudo systemctl restart NetworkManager`). Only the first source fetch needs the internet |
| 16 | monocular live stays in "initialising" when the rig is still | no pose for the whole run; frames take ~100 ms (the initialiser extracts 5x the features) | the rig needs real sideways motion with parallax; a live GUI must say so while initialising |
| 17 | trajectory writers crash if nothing was ever tracked | segmentation fault right after `Shutdown` (upstream reads the first keyframe without checking) | `mono_live` saves only if it tracked at least once |
| 18 | my own helper scripts matched themselves | `pgrep -f name` found the ssh command line that contained `name`, so "already running" / "still running" were wrong | use anchored patterns (`^python3 stereo_capture`) or `[b]racket` patterns |

Worked without change on aarch64: `wsl_fixes.patch` (C++17, int counter, shutdown fix), g2o, DBoW2, Pangolin v0.8, the vocabulary unpack.
Result of the first run: `stereo_0009`, headless, 860 of 917 frames tracked (94 %) - the same as in WSL (95 %).

## Live SLAM (how it is wired)
```
camera --GStreamer--> stereo_capture.py (preview, recording, IMU)
                          |  SlamFeed: newest gray frame -> /dev/shm/rig_slam_cam0 (seqlock, 30 fps)
                          v
                      mono_live (C++, ORB-SLAM3 monocular, skips frames when slower than the camera)
                          |  UDP localhost:5011  "P t x y z qx qy qz qw" | "S state fps ms skipped" | "N" (new map)
                          v
                      SlamRelay in stereo_capture.py  <--- RigCapture polls "slam poll N" over the control channel (UDP 5010)
```
The PC polls (a request always gets its reply through the PC's firewall); nothing needs an incoming port on the PC.
`slam on` / `slam off` start and stop `mono_live`; the settings file is installed by the capture program if missing; the path is
saved in `~/orbslam/live_out/` as EuRoC `f_app_N.txt` / `kf_app_N.txt`. Recording and SLAM are independent.

## Status
- Done: build, headless run, speed test, camera-free feed test, live camera test, RigCapture 1.1 with the SLAM button and the 3D view
  (shown working on the Orin NX), `build_gui.bat` from scratch on the PC.
- Next: monocular-inertial with the Cube IMU (the calibration is in `kalibr_noros/results`), install on a second Jetson to prove this page.
