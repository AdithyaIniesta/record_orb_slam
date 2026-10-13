#!/bin/bash
# Run ORB-SLAM3 monocular-INERTIAL on a RigCapture recording (camera cam0 + Cube IMU).
#
#   ./run_mono_inertial.sh RECORDING_DIR SETTINGS_INERTIAL.yaml [TAG] [START_S END_S]
#
# SETTINGS_INERTIAL.yaml comes from make_inertial_settings.py. The IMU stamps are moved by the Kalibr time offset
# (env SHIFT_S, default 0.012084: t_imu = t_cam + shift, so IMU stamps are moved back by it).
# START_S END_S run only that part of the recording (seconds from the first frame); the recording is not changed.
# Output (RECORDING_DIR/orbslam_TAG/): f_TAG.txt, kf_TAG.txt (EuRoC: t_ns x y z qx qy qz qw), run.log. The scale is metric.
# Env: ORB (install dir), TIMEOUT seconds (default 3600). The EuRoC example always opens the viewer, so a display is needed.
set -euo pipefail
REC="$(cd "${1:?usage: $0 RECORDING_DIR SETTINGS.yaml [TAG] [START_S END_S]}" && pwd)"
SET="$(cd "$(dirname "${2:?settings yaml}")" && pwd)/$(basename "$2")"
TAG="${3:-inertial}"; S="${4:-0}"; E="${5:-1000000}"
SHIFT_S="${SHIFT_S:-0.012084}"
if [ -z "${ORB:-}" ]; then
  for d in /opt/orbslam3 "$HOME/orbslam/orbslam3"; do [ -x "$d/Examples/Monocular-Inertial/mono_inertial_euroc" ] && ORB="$d" && break; done
fi
ORB="${ORB:-/opt/orbslam3}"
[ -x "$ORB/Examples/Monocular-Inertial/mono_inertial_euroc" ] || { echo "mono_inertial_euroc not built in $ORB"; exit 1; }
export LD_LIBRARY_PATH="$HOME/orbslam/local/lib:${LD_LIBRARY_PATH:-}"
W="$(mktemp -d /tmp/inertial_XXXXXX)"; mkdir -p "$W/mav0/cam0" "$W/mav0/imu0"
ln -s "$REC/mav0/cam0/data" "$W/mav0/cam0/data"
t0=$(head -1 "$REC/times.txt")
awk -v t0="$t0" -v s="$S" -v e="$E" '($1-t0)/1e9>=s && ($1-t0)/1e9<e' "$REC/times.txt" > "$W/times.txt"
t1=$(head -1 "$W/times.txt"); t2=$(tail -1 "$W/times.txt")
# IMU on the camera clock, from 1 s before the first frame to 1 s after the last
awk -F, -v sh="$SHIFT_S" -v a="$t1" -v b="$t2" 'BEGIN{OFS=","} /^#/ {print; next} { t=$1-sh*1e9; if (t>=a-1e9 && t<=b+1e9) { $1=sprintf("%.0f",t); print } }' \
    "$REC/mav0/imu0/data.csv" > "$W/mav0/imu0/data.csv"
echo "frames $(wc -l < "$W/times.txt"), imu rows $(wc -l < "$W/mav0/imu0/data.csv"), shift ${SHIFT_S}s"
OUT="$REC/orbslam_$TAG"; mkdir -p "$OUT"; cd "$W"
# viewer: WSLg needs X11 + software GL (the default window crashes)
unset WAYLAND_DISPLAY; export DISPLAY="${DISPLAY:-:0}" PANGOLIN_WINDOW_URI=x11:// LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
timeout "${TIMEOUT:-3600}" "$ORB/Examples/Monocular-Inertial/mono_inertial_euroc" "$ORB/Vocabulary/ORBvoc.txt" "$SET" "$W" "$W/times.txt" "$TAG" > "$OUT/run.log" 2>&1 || echo "(ORB-SLAM3 exit status $?; its log is $OUT/run.log)"
cp -f "$W"/f_"$TAG".txt "$W"/kf_"$TAG".txt "$OUT"/ 2>/dev/null || true
python3 - "$W/times.txt" "$OUT" "$TAG" <<'PY'
import os, sys
times, out, tag = sys.argv[1:4]
frames = sum(1 for l in open(times) if l.strip())
f = out + "/f_%s.txt" % tag
poses = sum(1 for l in open(f) if l.strip()) if os.path.exists(f) else 0
print("%s: %d of %d frames have a pose (%.0f %%) -> %s" % (tag, poses, frames, 100.0 * poses / max(frames, 1), out))
PY
rm -rf "$W"
