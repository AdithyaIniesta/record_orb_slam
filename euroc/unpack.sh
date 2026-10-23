#!/usr/bin/env bash
# Unpack EuRoC Vicon Room zips into a WSL-native directory for fast image I/O.
# Idempotent: skips sequences whose mav0/ already exists.
set -euo pipefail

SRC="/mnt/c/Users/admin/Downloads/vicon_room1/vicon_room1"
DST="${EUROC_DATA:-$HOME/euroc_data}"
mkdir -p "$DST"

for seq in V1_01_easy V1_02_medium V1_03_difficult; do
  out="$DST/$seq"
  if [[ -d "$out/mav0/cam0/data" ]]; then
    echo "[skip] $seq already unpacked at $out"
    continue
  fi
  zip="$SRC/$seq/$seq.zip"
  if [[ ! -f "$zip" ]]; then
    echo "[warn] missing $zip" >&2
    continue
  fi
  echo "[unpack] $seq -> $out"
  mkdir -p "$out"
  unzip -q "$zip" -d "$out"
done

echo "Done. Data root: $DST"

# ORB-SLAM3's shipped EuRoC_TimeStamps are CRLF, which silently breaks imread
# (the appended \r makes PNG paths wrong, and the terminal hides it). Build a
# clean LF copy next to this script.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TS_OUT="$HERE/timestamps"
mkdir -p "$TS_OUT"
for name in V101 V102 V103; do
  tr -d '\r' < "/opt/orbslam3/Examples/Monocular/EuRoC_TimeStamps/${name}.txt" > "$TS_OUT/${name}.txt"
done
echo "Timestamps (LF) in $TS_OUT"
