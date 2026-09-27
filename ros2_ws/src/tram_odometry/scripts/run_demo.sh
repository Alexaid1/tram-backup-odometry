#!/usr/bin/env bash
# One-command demo: play a bag through tram_odometry_node, record results, print metrics.
#   ros2 run tram_odometry run_demo.sh <bag_dir> [out_dir] [rate]
# Requires a sourced workspace (source install/setup.bash).
set -euo pipefail

BAG="${1:?usage: run_demo.sh <bag_dir> [out_dir] [rate]}"
OUT="${2:-/tmp/tram_odometry_demo}"
RATE="${3:-1.0}"
mkdir -p "$OUT"

cleanup() {
  kill "${NODE_PID:-}" "${REC_PID:-}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "[demo] starting tram_odometry_node"
ros2 run tram_odometry tram_odometry_node --ros-args \
  --params-file "$(ros2 pkg prefix tram_odometry)/share/tram_odometry/config/params.yaml" \
  > "$OUT/node.log" 2>&1 &
NODE_PID=$!
echo "[demo] starting result_recorder -> $OUT"
ros2 run tram_odometry result_recorder.py --ros-args -p out_dir:="$OUT" > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 3

echo "[demo] playing $BAG (rate $RATE)"
ros2 bag play "$BAG" --rate "$RATE"

echo "[demo] waiting for the recorder to flush"
wait "$REC_PID" 2>/dev/null || true
kill "$NODE_PID" 2>/dev/null || true

echo "[demo] metrics"
ros2 run tram_odometry evaluate_run.py "$OUT"
echo "[demo] node log: $OUT/node.log ; plots and metrics.json in $OUT"
