#!/usr/bin/env bash
# Run one ORB-SLAM3 EuRoC trial: mode x sequence.
# Usage: ./run_one.sh <mode> <seq>
#   mode: mono | mono_inertial | stereo | stereo_inertial
#   seq : V1_01 | V1_02 | V1_03
#
# Writes trajectories into results/<seq>/<mode>/ and tees stderr/stdout to run.log.
# Trajectories are written by ORB-SLAM3 as f_<name>.txt and kf_<name>.txt in cwd.
set -euo pipefail

MODE="${1:?mode required}"
SEQ="${2:?seq required}"

ORB=/opt/orbslam3
VOCAB="$ORB/Vocabulary/ORBvoc.txt"
DATA="${EUROC_DATA:-$HOME/euroc_data}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# LF-sanitized copy of ORB-SLAM3's EuRoC timestamps (see unpack.sh — the shipped
# files are CRLF, which silently breaks imread by appending \r to the png path).
TS_DIR="$REPO_ROOT/timestamps"

case "$SEQ" in
  V1_01) FOLDER=V1_01_easy;      TS=V101.txt ;;
  V1_02) FOLDER=V1_02_medium;    TS=V102.txt ;;
  V1_03) FOLDER=V1_03_difficult; TS=V103.txt ;;
  *) echo "unknown seq $SEQ" >&2; exit 2 ;;
esac
SEQ_DIR="$DATA/$FOLDER"
[[ -d "$SEQ_DIR/mav0" ]] || { echo "missing $SEQ_DIR/mav0 — run unpack.sh" >&2; exit 2; }
TS_FILE="$TS_DIR/$TS"
[[ -f "$TS_FILE" ]] || { echo "missing $TS_FILE — run unpack.sh" >&2; exit 2; }

OUT="$REPO_ROOT/results/$SEQ/$MODE"
mkdir -p "$OUT"
NAME="${SEQ}_${MODE}"

# Software-GL env so Pangolin can start under WSL if viewer is on; harmless when disabled.
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

case "$MODE" in
  mono)
    EX="$ORB/Examples/Monocular"; BIN="$EX/mono_euroc" ;;
  mono_inertial)
    EX="$ORB/Examples/Monocular-Inertial"; BIN="$EX/mono_inertial_euroc" ;;
  stereo)
    EX="$ORB/Examples/Stereo"; BIN="$EX/stereo_euroc" ;;
  stereo_inertial)
    EX="$ORB/Examples/Stereo-Inertial"; BIN="$EX/stereo_inertial_euroc" ;;
  *) echo "unknown mode $MODE" >&2; exit 2 ;;
esac

cd "$OUT"
"$BIN" "$VOCAB" "$EX/EuRoC.yaml" "$SEQ_DIR" "$TS_FILE" "$NAME" 2>&1 | tee run.log

echo "[done] $MODE $SEQ -> $OUT"
