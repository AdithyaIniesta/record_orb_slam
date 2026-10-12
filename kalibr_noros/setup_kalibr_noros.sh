#!/bin/bash
# Build ETH-ASL Kalibr (kalibr_calibrate_cameras) on plain Ubuntu - no ROS installed.
#
# Verified: Ubuntu 22.04 x86_64, Python 3.10, Boost 1.74, GCC 11 (WSL2).
# Not yet run on a Jetson. JetPack 5 = Ubuntu 20.04 / Python 3.8, JetPack 6 = Ubuntu 22.04 / Python 3.10;
# the patch picks the boost_python component from the interpreter version, so both should work.
#
# usage: sudo ./setup_kalibr_noros.sh [PREFIX]        (default PREFIX=/opt/kalibr_noros)
set -euo pipefail
PREFIX="${1:-/opt/kalibr_noros}"
HERE="$(cd "$(dirname "$0")" && pwd)"
KALIBR_COMMIT=1f60227442d25e36365ef5f72cd80b9666d73467   # patch is made against this commit
# The generated aslam_cv_serialization sources need ~1.5 GB of RAM per compiler process, so size
# the job count from memory, not just cores (16 jobs on a 7 GB machine got the compiler OOM-killed).
MEM_GB=$(awk '/MemAvailable/ {printf "%d", $2/1048576}' /proc/meminfo)
JOBS_BY_MEM=$(( MEM_GB * 2 / 3 )); [ "$JOBS_BY_MEM" -lt 1 ] && JOBS_BY_MEM=1
JOBS="${JOBS:-$(( JOBS_BY_MEM < $(nproc) ? JOBS_BY_MEM : $(nproc) ))}"
echo "Building with -j$JOBS (${MEM_GB} GB RAM available)"

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq build-essential cmake git python3-dev python3-pip python3-numpy python3-scipy \
  python3-matplotlib python3-yaml python3-opencv python3-igraph python3-cairo python3-pil \
  libeigen3-dev libboost-all-dev libopencv-dev libsuitesparse-dev libtbb-dev libblas-dev liblapack-dev \
  libv4l-dev pkg-config
# optional, only for kalibr_create_target_pdf (PyX needs TeX to typeset the label text)
apt-get install -y -qq --no-install-recommends texlive-latex-base poppler-utils || true
# catkin's python tooling, used purely as a build system (not ROS)
pip3 install -q catkin_pkg "empy==3.3.4" pyparsing python-dateutil pyx || \
  pip3 install -q --break-system-packages catkin_pkg "empy==3.3.4" pyparsing python-dateutil pyx

mkdir -p "$PREFIX/ws/src"
[ -d "$PREFIX/src" ] || git clone https://github.com/ethz-asl/kalibr.git "$PREFIX/src"
git -C "$PREFIX/src" checkout -q "$KALIBR_COMMIT"
if git -C "$PREFIX/src" apply --check "$HERE/kalibr-noros.patch" 2>/dev/null; then
  git -C "$PREFIX/src" apply "$HERE/kalibr-noros.patch"
else
  echo "patch already applied (or tree modified), skipping"
fi
touch "$PREFIX/src/catkin_simple/test/CATKIN_IGNORE"   # example packages that would confuse the workspace

ln -sfn ../../src "$PREFIX/ws/src/kalibr_repo"
[ -d "$PREFIX/ws/src/catkin" ] || git clone -q --depth 1 -b noetic-devel https://github.com/ros/catkin.git "$PREFIX/ws/src/catkin"

cd "$PREFIX/ws"
./src/catkin/bin/catkin_make_isolated --merge --only-pkg-with-deps kalibr \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j"$JOBS"

# launcher: kalibr-run <tool> [args]   e.g.  kalibr-run kalibr_calibrate_cameras --help
cat > "$PREFIX/kalibr-run" <<EOF
#!/bin/bash
tool="\$1"; shift
args=("\$@")
set --
source "$PREFIX/ws/devel_isolated/setup.bash"
export MPLBACKEND=\${MPLBACKEND:-Agg}
exec "$PREFIX/ws/devel_isolated/lib/kalibr/\$tool" "\${args[@]}"
EOF
chmod +x "$PREFIX/kalibr-run"
echo
echo "Done. Try:  $PREFIX/kalibr-run kalibr_calibrate_cameras --help"
