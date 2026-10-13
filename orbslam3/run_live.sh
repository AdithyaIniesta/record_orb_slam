#!/bin/bash
# Live ORB-SLAM3 (monocular, boresight camera) on the Jetson. One command, Ctrl+C stops everything.
#
#   ./run_live.sh [SECONDS]      (default: runs until Ctrl+C)
#
# 1. starts the capture program with the SLAM feed (and stops a running capture program first, e.g. the one RigCapture started)
# 2. starts mono_live; it prints one line per second: speed, tracking time, skipped frames, state
#    Hold the rig and move it slowly SIDEWAYS (not only turning) until it says TRACKING.
# 3. Ctrl+C: SLAM stops and saves f_<name>.txt / kf_<name>.txt in ~/orbslam/live_out, the capture program is stopped.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ORB_HOME="${ORB_HOME:-$HOME/orbslam}"
CAPTURE="${CAPTURE:-$HERE/../rigcapture/jetson/stereo_capture.py}"; [ -f "$CAPTURE" ] || CAPTURE="$ORB_HOME/wip/rigcapture_jetson/stereo_capture.py"
SETTINGS="${SETTINGS:-$HERE/settings/orbslam_mono_boresight_live.yaml}"; [ -f "$SETTINGS" ] || SETTINGS="$ORB_HOME/wip/settings/orbslam_mono_boresight_live.yaml"
FEED=/dev/shm/rig_slam_cam0
OUT="$ORB_HOME/live_out"; mkdir -p "$OUT" "$ORB_HOME/logs"
SECS="${1:-0}"
[ -f "$CAPTURE" ] && [ -f "$SETTINGS" ] && [ -x "$ORB_HOME/apps/mono_live" ] || { echo "missing: $CAPTURE / $SETTINGS / $ORB_HOME/apps/mono_live"; exit 1; }

if pgrep -f "^python3 .*stereo_capture" > /dev/null; then
  echo "stopping the running capture program ..."
  python3 -c 'import socket; socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(b"quit", ("127.0.0.1", 5010))'
  for i in $(seq 1 10); do pgrep -f "^python3 .*stereo_capture" > /dev/null || break; sleep 1; done
  pgrep -f "^python3 .*stereo_capture" > /dev/null && { echo "it did not stop - stop it by hand (close its window or: pkill -f stereo_capture.py)"; exit 1; }
fi
rm -f "$FEED"
cd "$(dirname "$CAPTURE")"
nohup python3 "$CAPTURE" --left /dev/video0 --cameras 1 --dest 192.168.0.100 --left-port 5000 --control-port 5010 \
    --preview-width 640 --preview-fps 15 --stream-kbps 3000 --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=40 --set-ctrl gain=1 \
    --record-fps 30 --imu-port auto --imu-rate 100 --slam-feed "$FEED" --slam-feed-fps 30 > "$ORB_HOME/logs/capture_live.log" 2>&1 &
CAP=$!
trap 'kill $CAP 2>/dev/null; sleep 1; echo "capture program stopped"' EXIT
for i in $(seq 1 20); do [ -e "$FEED" ] && break; sleep 0.5; done
[ -e "$FEED" ] || { echo "the camera feed did not start - see $ORB_HOME/logs/capture_live.log"; exit 1; }

n=$(ls "$OUT"/f_live_*.txt 2>/dev/null | wc -l); NAME="live_$((n + 1))"
echo ">>> SLAM starting (loading the vocabulary takes ~10 s). Move the rig slowly SIDEWAYS. Ctrl+C to stop. <<<"
cd "$OUT"
"$ORB_HOME/apps/mono_live" "$ORB_HOME/orbslam3/Vocabulary/ORBvoc.txt" "$SETTINGS" --shm "$FEED" --name "$NAME" ${SECS:+--seconds $SECS} 2>&1 | grep -a --line-buffered "\[live\]\|feed:"
ls -la "$OUT"/*"$NAME"* 2>/dev/null
