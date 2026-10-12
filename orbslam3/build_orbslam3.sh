#!/bin/bash
# Build ORB-SLAM3 (monocular / stereo / inertial examples) on Ubuntu 22.04 (verified in WSL2; Jetson not tried yet).
# usage (as root):  ./build_orbslam3.sh       -> clones upstream ORB-SLAM3 (pinned commit), applies wsl_fixes.patch, builds in /opt/orbslam3
# The build lives on the Linux filesystem (much faster than /mnt/c). Takes 20-40 min and several GB of RAM.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ORB_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4        # upstream commit our patch was made against
SRC="${ORB_SRC:-/opt/ORB_SLAM3_upstream}"
[ -d "$SRC/.git" ] || git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$SRC"
git -C "$SRC" checkout -q "$ORB_COMMIT"
DST=/opt/orbslam3
PANGOLIN_TAG=v0.8
# ORB-SLAM3 compiles several large files; keep RAM use sane (see the Kalibr build, where -j16 OOM-killed g++).
MEM_GB=$(awk '/MemAvailable/ {printf "%d", $2/1048576}' /proc/meminfo)
JOBS=$(( MEM_GB * 2 / 3 )); [ "$JOBS" -lt 1 ] && JOBS=1; [ "$JOBS" -gt "$(nproc)" ] && JOBS=$(nproc)
echo "Using -j$JOBS (${MEM_GB} GB RAM available)"

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq build-essential cmake git pkg-config libeigen3-dev libopencv-dev libssl-dev \
  libgl1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev wayland-protocols libepoxy-dev \
  libboost-serialization-dev libboost-system-dev python3-dev

# Pangolin (viewer) is not in apt
if [ ! -f /usr/local/lib/libpango_core.so ]; then
  [ -d /opt/Pangolin ] || git clone -q --depth 1 -b "$PANGOLIN_TAG" https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin
  cmake -S /opt/Pangolin -B /opt/Pangolin/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF
  cmake --build /opt/Pangolin/build -j"$JOBS"
  cmake --install /opt/Pangolin/build
  ldconfig
fi

mkdir -p "$DST"
rsync -a --delete --exclude '.git' --exclude 'build' --exclude 'lib' "$SRC/" "$DST/" 2>/dev/null || { cp -a "$SRC/." "$DST/"; rm -rf "$DST/.git"; }
cd "$DST"

sed -i 's/-std=c++11/-std=c++14/g' Thirdparty/DBoW2/CMakeLists.txt Thirdparty/g2o/CMakeLists.txt
# wsl_fixes.patch: C++17 for Pangolin 0.8, int instead of bool counter in LoopClosing.h, and System::Shutdown() waiting
# for the viewer/mapping threads. (CRLF line endings are stripped first in case the source came from a Windows checkout.)
sed -i 's/\r$//' CMakeLists.txt include/LoopClosing.h src/System.cc
if patch -p1 --dry-run -s < "$HERE/wsl_fixes.patch" >/dev/null 2>&1; then
  patch -p1 -s < "$HERE/wsl_fixes.patch"
else
  echo "wsl_fixes.patch already applied (or tree differs), skipping"
fi

# vocabulary
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary

for t in Thirdparty/DBoW2 Thirdparty/g2o; do
  cmake -S "$t" -B "$t/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$t/build" -j"$JOBS"
done
# Tracking.cc / Optimizer.cc / LocalMapping.cc need >2 GB each: a -j4 build on a 7 GB machine got cc1plus OOM-killed.
JOBS_MAIN=$(( MEM_GB / 3 )); [ "$JOBS_MAIN" -lt 1 ] && JOBS_MAIN=1; [ "$JOBS_MAIN" -gt 4 ] && JOBS_MAIN=4
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$JOBS_MAIN"
echo "BUILD_OK"; ls lib Examples/Monocular/mono_euroc Examples/Stereo/stereo_euroc
