#!/bin/bash
# Build Kalibr (camera and camera-IMU calibration, no ROS) on a Jetson (JetPack 5 = Ubuntu 20.04, aarch64).
# Needs NO root, except one apt line that is checked first and printed if anything is missing.
#
#   ./setup_kalibr_jetson.sh            build into ~/kalibr (override: KALIBR_HOME=...), about 30-60 min on an Orin NX
#   ./setup_kalibr_jetson.sh --check    only check the system packages and print the plan
#   ./setup_kalibr_jetson.sh --prepare  everything except the long compile (fetch, patch, launcher) - for testing
#
# OpenCV: the CUDA build in /opt/opencv-4.10.0-cuda (the one jetson-tracking-perception uses) when it exists, so the whole
# machine uses one OpenCV; otherwise JetPack's own 4.5.4. Force one with OPENCV_DIR=<dir with OpenCVConfig.cmake>.
# If the compile fails with the CUDA OpenCV (Kalibr was written for older OpenCV versions), run again with
#   OPENCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 ./setup_kalibr_jetson.sh
#
# Result:  ~/kalibr/kalibr-run <tool> ...   e.g.  ~/kalibr/kalibr-run kalibr_calibrate_imu_camera --help
#          ./calibrate_imu_camera.sh ~/recordings/stereo_NNNN cam0       (finds ~/kalibr by itself)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-build}"
KALIBR_COMMIT=1f60227442d25e36365ef5f72cd80b9666d73467        # the patch is made against this commit
KALIBR_HOME="${KALIBR_HOME:-$HOME/kalibr}"
SRC_GIT="$KALIBR_HOME/src/kalibr.git"
SRC="$KALIBR_HOME/src/kalibr"
WS="$KALIBR_HOME/ws"
LOGS="$KALIBR_HOME/logs"
MA="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || echo aarch64-linux-gnu)"

# ---- which OpenCV --------------------------------------------------------------------------------------------------
CUDA_CV=/opt/opencv-4.10.0-cuda
if [ -z "${OPENCV_DIR:-}" ]; then
  if [ -f "$CUDA_CV/lib/cmake/opencv4/OpenCVConfig.cmake" ]; then OPENCV_DIR="$CUDA_CV/lib/cmake/opencv4"
  else OPENCV_DIR="/usr/lib/$MA/cmake/opencv4"; fi
fi
[ -f "$OPENCV_DIR/OpenCVConfig.cmake" ] || { echo "OpenCV not found in $OPENCV_DIR (set OPENCV_DIR=...)"; exit 2; }
OPENCV_PREFIX="$(cd "$OPENCV_DIR/../../.." && pwd)"                 # .../opencv-4.10.0-cuda or /usr
CV_PY=""; EXTRA_LD=""; EXTRA_PY=""
if [ "$OPENCV_PREFIX" != "/usr" ]; then
  EXTRA_LD="$OPENCV_PREFIX/lib"
  CV_PY="$(find "$OPENCV_PREFIX" -name 'cv2*.so' -not -path '*/proc/*' 2>/dev/null | head -1 || true)"
  [ -n "$CV_PY" ] && EXTRA_PY="$(dirname "$CV_PY")"
fi
echo "OpenCV: $OPENCV_DIR  (python bindings of it: ${CV_PY:-none - Kalibr's python would use JetPack's cv2})"
[ -n "$EXTRA_PY" ] && export PYTHONPATH="$EXTRA_PY:${PYTHONPATH:-}"
[ -n "$EXTRA_LD" ] && export LD_LIBRARY_PATH="$EXTRA_LD:${LD_LIBRARY_PATH:-}"

# ---- system packages: the only step that needs sudo ---------------------------------------------------------------
need=()
chk () { [ -e "$1" ] || need+=("$2"); }
chk /usr/include/boost/python.hpp libboost-all-dev
chk /usr/include/boost/serialization/serialization.hpp libboost-all-dev
chk /usr/include/suitesparse/cholmod.h libsuitesparse-dev
chk /usr/include/tbb/tbb.h libtbb-dev
chk /usr/include/eigen3/Eigen/Core libeigen3-dev
chk "/usr/lib/$MA/libblas.so" libblas-dev
chk "/usr/lib/$MA/liblapack.so" liblapack-dev
chk /usr/include/libv4l2.h libv4l-dev
python3 -c "import sysconfig,os,sys; sys.exit(0 if os.path.exists(os.path.join(sysconfig.get_paths()['include'],'Python.h')) else 1)" 2>/dev/null || need+=(python3-dev)
for pair in numpy:python3-numpy scipy:python3-scipy matplotlib:python3-matplotlib yaml:python3-yaml igraph:python3-igraph cairo:python3-cairo PIL:python3-pil cv2:python3-opencv; do
  python3 -c "import ${pair%%:*}" 2>/dev/null || need+=("${pair##*:}")
done
command -v pip3 >/dev/null || need+=(python3-pip)
for t in cmake git patch pkg-config tar; do command -v $t >/dev/null || need+=($t); done
if [ ${#need[@]} -gt 0 ]; then
  uniq_need="$(printf '%s\n' "${need[@]}" | sort -u | tr '\n' ' ')"
  echo "Missing system packages. Run this once (asks for the sudo password), then start this script again:"
  echo "  sudo apt install -y $uniq_need"
  exit 2
fi
echo "system packages: all present"
getent hosts github.com > /dev/null || { echo "github.com does not resolve (the Jetson's DNS is empty after a reboot): run once  sudo resolvconf -u  and try again"; [ "$MODE" = "--check" ] || exit 2; }
MEM_GB=$(awk '/MemAvailable/ {printf "%d", $2/1048576}' /proc/meminfo)
JOBS=$(( MEM_GB / 2 )); [ "$JOBS" -lt 1 ] && JOBS=1; [ "$JOBS" -gt "$(nproc)" ] && JOBS=$(nproc)    # the generated sources need ~1.5 GB per compiler
echo "plan: Kalibr $KALIBR_COMMIT -> $KALIBR_HOME, -j$JOBS (${MEM_GB} GB RAM free)"
[ "$MODE" = "--check" ] && exit 0

mkdir -p "$KALIBR_HOME/src" "$LOGS" "$WS/src"
# python build helpers for catkin (used only as a build system, not ROS), into the user's home
python3 -m pip install --user -q catkin_pkg "empy==3.3.4" pyparsing python-dateutil > "$LOGS/pip.log" 2>&1 || { tail -5 "$LOGS/pip.log"; exit 1; }

# ---- source: the pinned commit only, kept as a bare repo; exported and patched fresh every time ----------------------
if [ ! -d "$SRC_GIT" ]; then
  git init -q --bare "$SRC_GIT"; git --git-dir="$SRC_GIT" remote add origin https://github.com/ethz-asl/kalibr.git
fi
git --git-dir="$SRC_GIT" cat-file -e "$KALIBR_COMMIT" 2>/dev/null || git --git-dir="$SRC_GIT" fetch -q --depth 1 origin "$KALIBR_COMMIT"
rm -rf "$SRC"; mkdir -p "$SRC"
git --git-dir="$SRC_GIT" archive "$KALIBR_COMMIT" | tar -x -C "$SRC"
( cd "$SRC" && patch -p1 -s < "$HERE/kalibr-noros.patch" )
mkdir -p "$SRC/catkin_simple/test" && touch "$SRC/catkin_simple/test/CATKIN_IGNORE"      # example packages that would confuse the workspace
ln -sfn ../../src/kalibr "$WS/src/kalibr_repo"
[ -d "$WS/src/catkin" ] || git clone -q --depth 1 -b noetic-devel https://github.com/ros/catkin.git "$WS/src/catkin"

# ---- launcher ----------------------------------------------------------------------------------------------------------
cat > "$KALIBR_HOME/kalibr-run" <<EOF
#!/bin/bash
# kalibr-run <tool> [args]    e.g.  kalibr-run kalibr_calibrate_imu_camera --help
tool="\$1"; shift
args=("\$@")
set --
${EXTRA_LD:+export LD_LIBRARY_PATH="$EXTRA_LD:\${LD_LIBRARY_PATH:-}"}
${EXTRA_PY:+export PYTHONPATH="$EXTRA_PY:\${PYTHONPATH:-}"}
source "$WS/devel_isolated/setup.bash"
export MPLBACKEND=\${MPLBACKEND:-Agg}
exec "$WS/devel_isolated/lib/kalibr/\$tool" "\${args[@]}"
EOF
chmod +x "$KALIBR_HOME/kalibr-run"
[ "$MODE" = "--prepare" ] && { echo "prepared (no compile): $SRC patched, launcher $KALIBR_HOME/kalibr-run"; exit 0; }

# ---- build ---------------------------------------------------------------------------------------------------------
cd "$WS"
if ! ./src/catkin/bin/catkin_make_isolated --merge --only-pkg-with-deps kalibr \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 -DOpenCV_DIR="$OPENCV_DIR" \
      --make-args -j"$JOBS" > "$LOGS/build.log" 2>&1; then
  grep -E "error|Error" "$LOGS/build.log" | head -8
  echo "BUILD FAILED - full log: $LOGS/build.log"
  echo "With the CUDA OpenCV this is often an OpenCV API difference; try:  OPENCV_DIR=/usr/lib/$MA/cmake/opencv4 $0"
  exit 1
fi
grep -q "Linking CXX shared library" "$LOGS/build.log" || { echo "nothing was compiled (stale objects? the device clock is unreliable) - remove $WS/build_isolated and run again"; exit 1; }

# ---- self-test: tools start, and only ONE OpenCV is loaded (two versions in one process crash) -------------------------------
"$KALIBR_HOME/kalibr-run" kalibr_calibrate_imu_camera --help > "$LOGS/selftest.log" 2>&1 && echo "kalibr_calibrate_imu_camera --help: OK" || { tail -8 "$LOGS/selftest.log"; echo "SELF-TEST FAILED (see above)"; exit 1; }
"$KALIBR_HOME/kalibr-run" kalibr_calibrate_cameras --help > /dev/null 2>&1 && echo "kalibr_calibrate_cameras --help: OK"
ENV_PY="$(mktemp)"; cat > "$ENV_PY" <<'PY'
import re, sys
import cv2, aslam_cv                      # Kalibr's C++ python module and the python cv2 in ONE process
libs = set(re.findall(r"libopencv_core\.so(?:\.[0-9.]+)?", open("/proc/self/maps").read()))
print("python cv2", cv2.__version__, "| libopencv_core loaded:", ", ".join(sorted(libs)) or "?")
sys.exit(1 if len({l.split(".so.")[-1] for l in libs if ".so." in l}) > 1 else 0)
PY
if ( export LD_LIBRARY_PATH="$EXTRA_LD:${LD_LIBRARY_PATH:-}"; export PYTHONPATH="$EXTRA_PY:${PYTHONPATH:-}"; source "$WS/devel_isolated/setup.bash"; python3 "$ENV_PY" ); then
  echo "OpenCV: one version in the process - fine"
else
  echo "WARNING: two different OpenCV versions are loaded together (the C++ part of Kalibr and python's cv2). Calibration may crash."
  echo "         Fix: build against the system OpenCV:  OPENCV_DIR=/usr/lib/$MA/cmake/opencv4 $0"
fi
rm -f "$ENV_PY"
echo "KALIBR_OK -> $KALIBR_HOME/kalibr-run"
echo "Use:  $HERE/calibrate_imu_camera.sh ~/recordings/stereo_NNNN cam0"
