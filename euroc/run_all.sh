#!/usr/bin/env bash
# Run all four modes across the requested sequences (default: V1_01 V1_02).
# Each trial is independent; one failing trial does not stop the rest.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEQS=("${@:-V1_01 V1_02}")
MODES=(mono mono_inertial stereo stereo_inertial)

for seq in ${SEQS[@]}; do
  for mode in "${MODES[@]}"; do
    echo "=== $seq / $mode ==="
    bash "$HERE/run_one.sh" "$mode" "$seq" || echo "[fail] $seq $mode (continuing)"
  done
done
