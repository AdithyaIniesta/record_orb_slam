#!/bin/bash
# Run ORB-SLAM3 monocular on a RigCapture recording and print how much of it was tracked.
#
#   ./run_mono.sh RECORDING_DIR SETTINGS.yaml [TAG]
#
#   settings/orbslam_mono_boresight.yaml        boresight 6 mm camera, recording saved at 60 fps
#   settings/orbslam_mono_boresight_3000.yaml   same, recording saved at 30 fps
#   settings/orbslam_mono_depression.yaml       depression 8 mm camera (cam0 of a recording made with that camera only)
# Output (in RECORDING_DIR/orbslam_TAG/): f_TAG.txt (every frame pose), kf_TAG.txt (keyframes), run.log  [EuRoC format: t_ns x y z qx qy qz qw]
# Monocular without the IMU: the scale is arbitrary. Needs real sideways motion to start (initialise).
# Env: ORB (install dir; default /opt/orbslam3 (WSL) or ~/orbslam3 (Jetson)), TIMEOUT seconds (default 600),
#      VIEWER=1 shows the ORB-SLAM3 windows (needs a display), default is headless
set -euo pipefail
REC="$(cd "${1:?usage: $0 RECORDING_DIR SETTINGS.yaml [TAG]}" && pwd)"
SET="$(cd "$(dirname "${2:?settings yaml}")" && pwd)/$(basename "$2")"
TAG="${3:-$(basename "$REC")}"
if [ -z "${ORB:-}" ]; then
  for d in /opt/orbslam3 "$HOME/orbslam/orbslam3"; do [ -x "$d/Examples/Monocular/mono_euroc" ] && ORB="$d" && break; done
fi
ORB="${ORB:-/opt/orbslam3}"
[ -x "$ORB/Examples/Monocular/mono_euroc" ] || { echo "ORB-SLAM3 not built in $ORB - run orbslam3/build_orbslam3.sh (WSL) or build_orbslam3_jetson.sh (Jetson)"; exit 1; }
export LD_LIBRARY_PATH="$HOME/orbslam/local/lib:${LD_LIBRARY_PATH:-}"      # Pangolin of the Jetson build
OUT="$REC/orbslam_$TAG"; mkdir -p "$OUT"; cd "$OUT"
if [ -n "${VIEWER:-}" ]; then
  export ORB_VIEWER=1
  # WSLg: the default Wayland/EGL window crashes; X11 + software GL is stable. Harmless elsewhere.
  unset WAYLAND_DISPLAY
  export DISPLAY="${DISPLAY:-:0}" PANGOLIN_WINDOW_URI=x11:// LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
fi
timeout "${TIMEOUT:-600}" "$ORB/Examples/Monocular/mono_euroc" "$ORB/Vocabulary/ORBvoc.txt" "$SET" "$REC" "$REC/times.txt" "$TAG" > run.log 2>&1 || echo "(ORB-SLAM3 exit status $?; its log is $OUT/run.log)"
python3 - "$REC" "$OUT" "$TAG" <<'PY'
import os, sys
rec, out, tag = sys.argv[1:4]
frames = sum(1 for l in open(rec + "/times.txt") if l.strip())
f = out + "/f_%s.txt" % tag
poses = sum(1 for l in open(f) if l.strip()) if os.path.exists(f) else 0
kf = out + "/kf_%s.txt" % tag
kfs = sum(1 for l in open(kf) if l.strip()) if os.path.exists(kf) else 0
print("%s: %d of %d frames have a pose (%.0f %%), %d keyframes -> %s" % (tag, poses, frames, 100.0 * poses / max(frames, 1), kfs, out))
PY
