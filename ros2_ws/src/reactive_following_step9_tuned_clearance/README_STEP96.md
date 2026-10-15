# Step 9.6 — Reduced avoidance oscillation + sensor health diagnostic

This full Webots/ROS 2 project derives from **Step 9.5 directional markers**, which already passed front/rear camera classification tests. The LiDAR mounting, camera classifier, recognition markers, wheel physics, arena layouts, and the ROS 2 UDP bridge are unchanged.

## What changed

1. `ros2_controller/safe_logic.py`: use a *consistent* right-hand turn preference across `CORNER_CLEAR_TURN` and `AVOID_DIAGONAL` when the front corridor is narrow (front <1.25 m and a front diagonal <0.65 m). The previous `AVOID_DIAGONAL` could command +0.8 rad/s while `CORNER_CLEAR_TURN` commanded -0.8 rad/s on the next scan. Immediate critical side clearance may override this preference. No robot/target pose, target history, or persistent escape state is stored.
2. `ros2_controller/follow_improved.py`: add `/<robot>/sensor_health` (std_msgs/String) reporting `OK`, `NO_SNAPSHOT`, `STALE_SNAPSHOT`, or `INVALID_SNAPSHOT`. This topic is a diagnostic only and never affects robot decisions beyond the existing stop-on-bad-sensors failsafe.
3. `tests/test_avoidance_no_chatter.py`: synthetic boundary regression checks, including original head-on and base-curve behavior.

## Run the head-on diagnostic

1. Close old ROS 2 bridge/follower processes; open `worlds/diagnostic_head_on.wbt` in Webots and press Play.
2. In ROS 2 terminal 1:

   ```bat
   cd /d C:\dev\reactive_following_step9_avoidance_fixed
   python ros2_bridge\bridge_all.py --robots R1 R2
   ```

3. In ROS 2 terminal 2:

   ```bat
   cd /d C:\dev\reactive_following_step9_avoidance_fixed
   python ros2_controller\follow_improved.py --robots R1 R2
   ```

4. Observe that robots separate, then move clear of walls rather than rocking indefinitely between opposite yaw commands.

## If SENSOR_FAILSAFE occurs

Check while Webots is still in Play:

```bat
ros2 topic echo /R1/sensor_health
ros2 topic echo /R2/sensor_health
ros2 topic echo /R1/sim_time
ros2 topic echo /R2/sim_time
```

* `OK`: recent valid full camera + LiDAR snapshot.
* `NO_SNAPSHOT`: no sensor snapshot has arrived at the follower.
* `STALE_SNAPSHOT`: no snapshot in >0.25 s of real clock time; check ROS 2 bridge delivery.
* `INVALID_SNAPSHOT`: a message arrived but the full scan was invalid; if both robots show it, check whether Webots stopped transmitting and the bridge is publishing fallback values.
* If `/R1/sim_time` and `/R2/sim_time` stop increasing, Webots may be paused, a controller may have stopped, or packets may no longer be reaching the bridge. Inspect the Webots and bridge consoles.

The robot will remain stopped on missing or invalid readings; **do not disable this failsafe**.

## Validation limits

`python -m unittest discover -s tests -p test_*.py -q` runs 71 offline tests. These are geometry/algorithm unit tests, not Webots physics tests. The five-robot scenario has **not** been validated in this version. Run the two-robot diagnostic first, then the corner scene, then the three-robot chain, and only then five robots.
