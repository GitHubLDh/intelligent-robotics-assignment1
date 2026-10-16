#!/bin/bash
# Starts all 3 pieces of the reactive_following_step9_tuned_clearance system:
# Webots (the body), bridge_all.py (the translator), follow_improved.py (the brain).
# Usage: ./run_trains.sh [world_file_name] [-- follower args]
#   default world: big_no_obstacles.wbt
# Slide 7 experiments (identical initial conditions, vary the turning):
#   ./run_trains.sh -- --steering-gain 0      # no following
#   ./run_trains.sh -- --steering-gain 0.8    # weak turning
#   ./run_trains.sh -- --steering-gain 2.6    # strong turning
# Slide 8 extensions:  -- --avoid --slow
#   --constant-speed  spec-pure slide 4 baseline (no standoff: drives into
#                     the leader, which is what the collision metric shows)

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$REPO_DIR/ros2_ws/src/reactive_following_step9_tuned_clearance"
WORLD="big_no_obstacles.wbt"
if [ $# -gt 0 ] && [ "$1" != "--" ]; then WORLD="$1"; shift; fi
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
