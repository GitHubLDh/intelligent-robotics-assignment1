#!/bin/bash
# Starts all 3 pieces of the following_robots system:
# Webots (the body), bridge_all.py (the translator), reactive_follow.py (the brain).
# Usage: ./run_trains.sh [1-5 | world_file_name] [-- follower args]
#
# Starting layout: pass 1-5 to place the robots before launching.
#   1  ring        0.95 m circle, tangent  - starts already following
#   2  wide-ring   1.50 m circle, tangent  - must approach first
#   3  scattered   far apart, random headings - must search
#   4  column      single file, all facing +x - already a train
#   5  facing-in   1.20 m circle, pointed inward - worst case
# Omit it to leave the world's current poses untouched.
#
#   ./run_trains.sh 3                       # scattered start
#   ./run_trains.sh 1 -- --steering-gain 0  # ring start, no following
#
# Slide 7 experiments (identical initial conditions, vary the turning):
#   ./run_trains.sh 1 -- --steering-gain 0     # no following
#   ./run_trains.sh 1 -- --steering-gain 0.8   # weak turning
#   ./run_trains.sh 1 -- --steering-gain 2.6   # strong turning

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$REPO_DIR/ros2_ws/src/following_robots"
WORLD="big_no_obstacles.wbt"
LAYOUT=""
# A bare 1-5 selects a starting layout; anything else is a world file name.
if [ $# -gt 0 ] && [ "$1" != "--" ]; then
    if [[ "$1" =~ ^[1-5]$ ]]; then LAYOUT="$1"; else WORLD="$1"; fi
    shift
fi
[ "${1:-}" = "--" ] && shift
FOLLOWER_ARGS=("$@")

# Don't hardcode a distro - use whichever one is actually installed.
# If your shell already sources ROS2 on startup (check ~/.bashrc), this is a no-op.
source /opt/ros/*/setup.bash 2>/dev/null || true

LOG_DIR="$REPO_DIR/logs"
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
    echo ""
    echo "Stopping everything..."
    kill "${PIDS[@]}" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ -n "$LAYOUT" ]; then
    echo "[0/3] Setting starting layout $LAYOUT ..."
    # Must run before Webots opens the world, or it loads the old poses.
    python3 "$PKG_DIR/tools/set_start_layout.py" "$LAYOUT"
fi

echo "[1/3] Starting Webots with $WORLD ..."
webots --batch --mode=realtime "$PKG_DIR/worlds/$WORLD" > >(tee "$LOG_DIR/webots.log") 2>&1 &
PIDS+=($!)

sleep 5

echo "[2/3] Starting ROS2 bridge..."
python3 -u "$PKG_DIR/ros2_bridge/bridge_all.py" > >(tee "$LOG_DIR/bridge.log") 2>&1 &
PIDS+=($!)

sleep 2

echo "[3/3] Starting follower (the brain)..."
python3 -u "$PKG_DIR/ros2_controller/reactive_follow.py" "${FOLLOWER_ARGS[@]}" \
    > >(tee "$LOG_DIR/follower.log") 2>&1 &
PIDS+=($!)

echo ""
echo "All 3 running. Press Ctrl+C here to stop everything at once."
echo "Logs (overwritten fresh each run) in: $LOG_DIR/{webots,bridge,follower}.log"
wait
