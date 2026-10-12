#!/bin/bash
# Camera <-> IMU calibration of one RigCapture recording with Kalibr (no ROS).
#
#   ./calibrate_imu_camera.sh RECORDING_DIR [cam0|cam1] [TARGET.yaml]
#
#   RECORDING_DIR  a RigCapture recording (has mav0/cam0, mav0/imu0, times.txt, meta.json)
#   cam0           boresight 6 mm camera (default)      cam1  depression 8 mm camera (two-camera recordings)
#   TARGET.yaml    default inputs/aprilgrid_kalibr_6x4_50mm_A3.yaml  (put the MEASURED tag size in it)
#
# The board must stand still and be seen by the chosen camera; only the rig moves.
# Result: results/<recording>_<cam>/*-camchain-imucam.yaml  (T_cam_imu = IMU -> camera, timeshift_cam_imu: t_imu = t_cam + shift)
# Env: PREFIX (Kalibr install, default /opt/kalibr_noros), WORK (scratch, default /tmp/kalibr_work)
set -euo pipefail
REC="$(cd "${1:?usage: $0 RECORDING_DIR [cam0|cam1] [TARGET.yaml]}" && pwd)"
CAM="${2:-cam0}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TARGET="${3:-$HERE/inputs/aprilgrid_kalibr_6x4_50mm_A3.yaml}"
PREFIX="${PREFIX:-/opt/kalibr_noros}"
WORK="${WORK:-/tmp/kalibr_work}"
RUN="$PREFIX/kalibr-run"; [ -x "$RUN" ] || RUN="$PREFIX/run.sh"
[ -x "$RUN" ] || { echo "Kalibr not found in $PREFIX - run: sudo ./setup_kalibr_noros.sh"; exit 1; }
NAME="$(basename "$REC")_$CAM"
DS="$WORK/$NAME"
rm -rf "$DS"; mkdir -p "$DS/mav0/$CAM" "$DS/mav0/imu0"
ln -s "$REC/mav0/imu0/data.csv" "$DS/mav0/imu0/data.csv"

case "$CAM" in
  cam0) CAMCHAIN="$HERE/inputs/camchain_6mm_boresight.yaml"; ln -s "$REC/mav0/cam0/data" "$DS/mav0/cam0/data" ;;
  cam1) CAMCHAIN="$HERE/inputs/camchain_8mm_depression.yaml"
        # cam1 frames carry cam0's time stamp but were captured a few ms later (pairs.csv): restamp them with their own capture time
        python3 - "$REC" "$DS" <<'PY'
import os, sys
import csv
rec, ds = sys.argv[1], sys.argv[2]
os.makedirs(ds + "/mav0/cam1/data", exist_ok=True)
n = 0
with open(rec + "/pairs.csv") as f:
    for row in csv.DictReader(f):
        src = "%s/mav0/cam1/data/%s.png" % (rec, row["ts_ns"])
        if os.path.exists(src):
            os.symlink(src, "%s/mav0/cam1/data/%s.png" % (ds, row["right_mono_ns"])); n += 1
print("cam1 restamped from pairs.csv: %d frames" % n)
PY
        ;;
  *) echo "camera must be cam0 or cam1"; exit 1 ;;
esac

# IMU rate -> yaml and spline density (the pose spline must not have more knots per second than the IMU has samples)
RATE=$(python3 - "$REC" <<'PY'
import json, sys
try:
    hz = json.load(open(sys.argv[1] + "/meta.json"))["imu"]["measured_hz"]
except Exception:
    hz = 100.0
print(100 if hz >= 70 else 40)
PY
)
IMU="$HERE/inputs/imu_cube_${RATE}hz.yaml"
POSE_KNOTS=$RATE; BIAS_KNOTS=$((RATE / 2))
echo "recording $REC | $CAM | IMU $RATE Hz | spline knots $POSE_KNOTS/$BIAS_KNOTS per s | target $(basename "$TARGET")"

cd "$WORK"
"$RUN" kalibr_calibrate_imu_camera --bag "$DS" --bag-freq 20 --timeoffset-padding 0.08 \
    --pose-knots-per-second $POSE_KNOTS --bias-knots-per-second $BIAS_KNOTS \
    --cam "$CAMCHAIN" --imu "$IMU" --target "$TARGET" --dont-show-report 2>&1 | tr '\r' '\n' | grep -v "Progress" | tee "$WORK/$NAME.log" | tail -25

OUT="$HERE/results/$NAME"
mkdir -p "$OUT"
cp "$WORK/$NAME-camchain-imucam.yaml" "$WORK/$NAME-imu.yaml" "$WORK/$NAME-results-imucam.txt" "$OUT/" 2>/dev/null || { echo "Kalibr produced no result (see $WORK/$NAME.log)"; exit 1; }
cp "$WORK/$NAME.log" "$OUT/kalibr.log"
python3 - "$OUT/$NAME-camchain-imucam.yaml" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))["cam0"]; T = c["T_cam_imu"]
print("\nT_cam_imu (IMU -> camera):")
for r in T: print("  " + "  ".join("%9.5f" % v for v in r))
print("lever arm t_ci = (%.1f, %.1f, %.1f) mm | time shift %.2f ms (t_imu = t_cam + shift)" % (T[0][3]*1000, T[1][3]*1000, T[2][3]*1000, c["timeshift_cam_imu"]*1000))
PY
echo "saved in $OUT"
