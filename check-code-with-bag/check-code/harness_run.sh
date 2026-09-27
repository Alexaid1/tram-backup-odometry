#!/usr/bin/env bash
# Official-style check: metrics checker + solution launch + bag playback, with CPU/RAM sampling.
# Usage (inside the container, after colcon build): bash /workspace/harness_run.sh [rate]
# job control: background jobs get their own process group and do not ignore SIGINT
set -m
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
OUT=/workspace/harness_out
RATE="${1:-1.0}"
rm -rf "$OUT"; mkdir -p "$OUT"

ros2 run hackathon_solution_checker metrics > "$OUT/metrics.log" 2>&1 &
MET=$!
ros2 launch tram_odometry tram_odometry.launch.py > "$OUT/launch.log" 2>&1 &
LAU=$!
sleep 5

# sample the node process once per second: time %cpu rss_kb
( while true; do
    P=$(pgrep -f tram_odometry_node | head -1)
    [ -n "$P" ] && echo "$(date +%s) $(ps -o %cpu=,rss= -p "$P")" >> "$OUT/res.log"
    sleep 1
  done ) &
SMP=$!

ros2 bag play /workspace/bags/30618_88aea4d9 --rate "$RATE" > "$OUT/play.log" 2>&1
sleep 3
kill -- -$SMP
kill -INT -- -$MET; sleep 2
kill -INT -- -$LAU; sleep 3
kill -TERM -- -$MET -$LAU 2>/dev/null
echo "=== final metrics ==="
grep -E "metrics" "$OUT/metrics.log" | tail -2
echo "=== node resources (cpu% / rss MB): mean, max ==="
awk '{c+=$2; if($2>cm)cm=$2; if($3>rm)rm=$3; n++} END{printf "cpu mean %.1f%% max %.1f%%, rss max %.1f MB, n=%d\n", c/n, cm, rm/1024, n}' "$OUT/res.log"
echo "=== launch log (errors/warnings) ==="
grep -iE "error|warn|exception|died" "$OUT/launch.log" | head -20
echo DONE
