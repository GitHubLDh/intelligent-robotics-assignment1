# Step 9.8 — Consistent stateless corner recovery

This version modifies **only `ros2_controller/safe_logic.py`**, the affected regression tests, and documentation. It carries over the directional camera, LiDAR, Webots robot physics, bridge and ROS 2 shutdown handling from Step 9.7.

## Why this exists

In the Step 9.7 head-on test, R2 repeatedly changed yaw from -0.80 to +0.80 rad/s between `BRAKE_TURN`, `AVOID_DIAGONAL`, `AVOID_SIDE`, and `CORNER_CLEAR_TURN`. The previous 'single escape direction' calculation still included a range-dependent exception for a close right-side wall. As the scan changed, this changed the sign of the commanded yaw.

## Changes

- All obstacle-pivot and head-on avoidance modes use a **clockwise/right-hand** angular direction where a pivot is physically permitted. A side obstacle inside the 0.19m safety margin now causes a cautious straight backoff if the current rear scan is clear, otherwise `SIDE_BLOCKED` with zero movement. This sacrifices some recoverability in unusual configurations to avoid steering reversals and unsafe contact.
- `CLEAR_FORWARD_ARC` advances at 0.035m/s while turning right when current frontal, diagonal and side clearances meet explicit thresholds, instead of spinning in place until all diagonal rays exceed 1.10m.
- `BASE_CLEAR_BLEND` gradually returns angular velocity to each robot's original base curvature when the LiDAR shows improving forward space. No remembered target, previous heading, or avoidance direction is used.
- The communication failsafe is **unchanged**: if Webots is paused or telemetry stops, both robots correctly command zero. The earlier `RCLError` traceback came from the older Step 9.6 code, not the Step 9.7 controller.

## Test A — Targeted two-robot head-on and corner

1. Close all old ROS 2 bridges and follower scripts, and close the previous Webots world.
2. Open `worlds/diagnostic_head_on.wbt` in Webots and press Play.
3. Run the following commands in separate ROS 2 Pixi/lyrical Developer Command Prompts:

Terminal 1:

```bat
cd /d C:\dev\reactive_following_step9_stable_corner
python ros2_bridge\bridge_all.py --robots R1 R2
```

Terminal 2:

```bat
cd /d C:\dev\reactive_following_step9_stable_corner
python ros2_controller\follow_improved.py --robots R1 R2
```

Terminal 3 (optional):

```bat
ros2 topic echo /R2/follow_mode
```

Observe the robots for at least 30 seconds **of simulation time**. Check whether R2 separates from R1, escapes nearby walls without left/right shaking, and can resume movement. Use `/R2/contact` to identify physical contacts. Repeat after resetting the Webots world.

## Test B — If SENSOR_FAILSAFE occurs

In separate terminals:

```bat
ros2 topic echo /R2/bridge_health
ros2 topic echo /R2/sim_time
ros2 topic echo /R2/sensor_health
```

If `/R2/bridge_health` is `WEBOTS_TELEMETRY_STALE`, check whether Webots is paused, whether its robot controllers are still running, and whether Terminal 1 remains active. `/R2/sim_time` must keep increasing in real time while Play is active.

`STALE_SNAPSHOT` / `INVALID_SNAPSHOT` are expected after missing bridge data because the bridge publishes invalid safety snapshots on loss of Webots telemetry. Do not disable this failsafe to make the robots move.

## Assignment constraints

No GPS, no Supervisor, no absolute/global robot pose, no target ID or history, no robot-to-robot coordination, no persistent escape-direction memory. Uses only current local LiDAR / camera observations. Camera-based target selection and the safe following distance remain unchanged.

## Validation limitations

83 isolated Python tests passed in the packaged source. They exercise control decisions and rough geometric examples, **not Webots contact physics, actual multi-robot deadlocks, or UDP robustness on Windows**. The deterministic right-hand policy cannot guarantee escape from every concave corner under the memoryless/local-sensor restriction. A physically trapped robot must stop instead of turning or reversing into an obstacle.
