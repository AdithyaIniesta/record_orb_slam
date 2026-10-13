#!/bin/bash
# Build ORB-SLAM3 on a Jetson (JetPack 5 = Ubuntu 20.04, aarch64). Needs NO root, except for one apt line that is checked first.
#
#   ./build_orbslam3_jetson.sh            -> everything under ~/orbslam (override with ORB_HOME=...), about 20-40 min on an Orin NX
#
#   ~/orbslam/src/        upstream ORB-SLAM3 (pinned commit, shallow) and Pangolin sources
#   ~/orbslam/orbslam3/   patched, built ORB-SLAM3 (Examples/Monocular/mono_euroc, lib/libORB_SLAM3.so, ...)
#   ~/orbslam/local/      Pangolin install (no sudo)
#   ~/orbslam/apps/       our programs on top of it (mono_bench, mono_live)
#   ~/orbslam/logs/       all build logs
# Headless by default (viewer only with ORB_VIEWER=1, see viewer_env.patch). Problems met on the way: docs/orbslam3_jetson.md
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ORB_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4        # upstream commit our patches were made against
PANGOLIN_TAG=v0.8
ORB_HOME="${ORB_HOME:-$HOME/orbslam}"
SRC="${ORB_SRC:-$ORB_HOME/src/ORB_SLAM3.git}"
DST="${ORB_DST:-$ORB_HOME/orbslam3}"
LOCAL="${LOCAL_PREFIX:-$ORB_HOME/local}"
PANGOLIN_SRC="$ORB_HOME/src/Pangolin"
LOGS="$ORB_HOME/logs"
mkdir -p "$ORB_HOME/src" "$LOGS"
# JetPack's own OpenCV (4.5.4). ORB-SLAM3 needs >= 4.4. Some Jetsons also have a CUDA OpenCV in /opt - do not pick that one by accident.
OPENCV_DIR="${OPENCV_DIR:-/usr/lib/aarch64-linux-gnu/cmake/opencv4}"

# 1. system packages: the only step that needs sudo. Check, and stop with the exact command if something is missing.
need=()
chk () { [ -e "$1" ] || need+=("$2"); }
chk /usr/include/openssl/md5.h libssl-dev
chk /usr/include/boost/serialization/serialization.hpp libboost-serialization-dev
chk /usr/include/GL/glew.h libglew-dev
chk /usr/include/epoxy/gl.h libepoxy-dev
chk /usr/include/eigen3/Eigen/Core libeigen3-dev
chk /usr/include/GL/gl.h libgl1-mesa-dev
chk /usr/include/X11/Xlib.h libx11-dev
command -v cmake >/dev/null || need+=(cmake)
command -v git >/dev/null   || need+=(git)
command -v patch >/dev/null || need+=(patch)
command -v rsync >/dev/null || need+=(rsync)
if [ ${#need[@]} -gt 0 ]; then
  echo "Missing system packages. Run this once (asks for the sudo password), then start this script again:"
  echo "  sudo apt install -y ${need[*]}"
  exit 2
fi
[ -f "$OPENCV_DIR/OpenCVConfig.cmake" ] || { echo "OpenCV not found in $OPENCV_DIR (set OPENCV_DIR=...)"; exit 2; }

MEM_GB=$(awk '/MemAvailable/ {printf "%d", $2/1048576}' /proc/meminfo)
JOBS=$(nproc); JOBS_MAIN=$(( MEM_GB / 3 )); [ "$JOBS_MAIN" -lt 1 ] && JOBS_MAIN=1; [ "$JOBS_MAIN" -gt 4 ] && JOBS_MAIN=4
echo "OpenCV $(grep -m1 -o 'OpenCV_VERSION [0-9.]*' $OPENCV_DIR/OpenCVConfigVersion.cmake 2>/dev/null || echo ' ') | -j$JOBS (-j$JOBS_MAIN for ORB-SLAM3 itself, ${MEM_GB} GB RAM free)"

# 2. source: only the pinned commit, kept as a bare repository (~180 MB). A normal checkout is 1.6 GB, 1.3 GB of it TUM-VI
#    example data we never use.
if [ ! -d "$SRC" ]; then
  git init -q --bare "$SRC"; git --git-dir="$SRC" remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git
fi
git --git-dir="$SRC" cat-file -e "$ORB_COMMIT" 2>/dev/null || git --git-dir="$SRC" fetch -q --depth 1 origin "$ORB_COMMIT"

# 3. patched copy exported from it (without the TUM_IMU data). Fresh copy every time: file times on these devices are not
#    reliable, so make can wrongly think objects are up to date.
rm -rf "$DST"; mkdir -p "$DST"
git --git-dir="$SRC" archive "$ORB_COMMIT" | tar -x -C "$DST" --exclude='TUM_IMU'
cd "$DST"
sed -i 's/-std=c++11/-std=c++14/g' Thirdparty/DBoW2/CMakeLists.txt Thirdparty/g2o/CMakeLists.txt
for p in wsl_fixes.patch viewer_env.patch; do patch -p1 -s < "$HERE/$p"; done      # C++17 / shutdown fix / headless by default
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary

# 4. Pangolin (viewer library; linked even when the viewer is off) into $LOCAL, no sudo
if [ ! -e "$LOCAL/lib/libpango_core.so" ]; then
  [ -d "$PANGOLIN_SRC" ] || git clone -q --depth 1 -b "$PANGOLIN_TAG" https://github.com/stevenlovegrove/Pangolin.git "$PANGOLIN_SRC"
  cmake -S "$PANGOLIN_SRC" -B "$PANGOLIN_SRC/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_PANGOLIN_PYTHON=OFF -DCMAKE_INSTALL_PREFIX="$LOCAL" > "$LOGS/pangolin_cmake.log" 2>&1
  cmake --build "$PANGOLIN_SRC/build" -j"$JOBS" > "$LOGS/pangolin_make.log" 2>&1
  cmake --install "$PANGOLIN_SRC/build" > "$LOGS/pangolin_install.log" 2>&1
fi
export LD_LIBRARY_PATH="$LOCAL/lib:${LD_LIBRARY_PATH:-}"

# 5. ORB-SLAM3 and its third-party libraries
for t in Thirdparty/DBoW2 Thirdparty/g2o; do
  n=$(basename "$t")
  cmake -S "$t" -B "$t/build" -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="$OPENCV_DIR" > "$LOGS/${n}_cmake.log" 2>&1
  cmake --build "$t/build" -j"$JOBS" > "$LOGS/${n}_make.log" 2>&1 || { echo "$t failed, see $LOGS/${n}_make.log"; exit 1; }
done
mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="$OPENCV_DIR" -DCMAKE_PREFIX_PATH="$LOCAL" > "$LOGS/orbslam3_cmake.log" 2>&1 || { tail -20 "$LOGS/orbslam3_cmake.log"; exit 1; }
cmake --build build -j"$JOBS_MAIN" > "$LOGS/orbslam3_make.log" 2>&1 || { grep -E "error|Error" "$LOGS/orbslam3_make.log" | head; echo "see $LOGS/orbslam3_make.log"; exit 1; }
grep -q "Linking CXX shared library" "$LOGS/orbslam3_make.log" || { echo "nothing was compiled - stale objects? (check device clock)"; exit 1; }

# 6. our programs on top (mono_bench; mono_live follows)
cmake -S "$HERE/apps" -B "$ORB_HOME/apps" -DORB_SLAM3_ROOT="$DST" -DOpenCV_DIR="$OPENCV_DIR" -DCMAKE_PREFIX_PATH="$LOCAL" > "$LOGS/apps_cmake.log" 2>&1 || { tail -20 "$LOGS/apps_cmake.log"; exit 1; }
cmake --build "$ORB_HOME/apps" -j"$JOBS_MAIN" > "$LOGS/apps_make.log" 2>&1 || { grep -E "error" "$LOGS/apps_make.log" | head; exit 1; }
echo "BUILD_OK"; ls -la lib/libORB_SLAM3.so Examples/Monocular/mono_euroc Examples/Monocular-Inertial/mono_inertial_euroc Examples/Stereo/stereo_euroc "$ORB_HOME"/apps/mono_bench
echo "Run:  $HERE/run_mono.sh RECORDING_DIR $HERE/settings/orbslam_mono_boresight.yaml        (ORB=$DST)"
echo "Speed: LD_LIBRARY_PATH=$LOCAL/lib $ORB_HOME/apps/mono_bench $DST/Vocabulary/ORBvoc.txt SETTINGS.yaml RECORDING_DIR"
