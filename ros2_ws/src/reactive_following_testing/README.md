## Quick start on Windows (Webots + TWO ROS 2 terminals)

**Before starting:** Close the old Webots world and any Step 4 ROS 2 bridge,
controller, and manual `/R1/cmd_vel` publisher. Using the old bridge at the same
time would produce confusing results.

1. Extract to `C:\dev\reactive_following_step5` so `worlds`, `protos`, and
   `ros2_bridge` are directly under that folder.
2. Webots -> **File > Open World** ->
   `C:\dev\reactive_following_step5\worlds\reactive_following.wbt`.
   Press **Play**. Webots should start five `udp_diff_driver` controllers.
3. ROS 2 Terminal 1: open your existing ROS 2 / Pixi Developer Prompt (the
   shortcut you previously configured) and run:

   ```bat
   cd /d C:\dev\reactive_following_step5
   python ros2_bridge\bridge_all.py
   ```

   You should see `Connected to Webots R1`, `... R2`, ... `... R5` (order
   may differ). Leave this terminal running.
4. ROS 2 Terminal 2: open a **second** ROS 2 / Pixi Developer Prompt and run:

   ```bat
   cd /d C:\dev\reactive_following_step5
   python ros2_controller\follow_all.py
   ```

   Expect one startup line for each robot and subsequent `Ri mode: BASE_CURVE`
   and/or `Ri mode: FOLLOW`. Watch the robots move in Webots.
5. To stop autonomous movement, press **Ctrl+C in Terminal 2**. The ROS 2
   bridge has a command deadman timer, so all wheels should stop after roughly
   0.7 s without commands. It is also fine to pause/reset Webots between
   tests.

**Do not run any older `bridge_r1.py` or `follow_r1.py` with these scripts.**

## How the five robots are independent

Webots instantiates the **same robot model and Python controller** five times:

| Robot | Webots controller UDP port | ROS 2 command topic | Speed (m/s) | Base turn (rad/s) |
|:--|--:|:--|--:|--:|
| R1 | 15001 | `/R1/cmd_vel` | 0.16 | -0.20 |
| R2 | 15002 | `/R2/cmd_vel` | 0.14 | +0.24 |
| R3 | 15003 | `/R3/cmd_vel` | 0.12 | -0.16 |
| R4 | 15004 | `/R4/cmd_vel` | 0.15 | +0.18 |
| R5 | 15005 | `/R5/cmd_vel` | 0.13 | -0.26 |

All five have one Webots recognition camera, dedicated UDP bridge socket,
**independent ROS 2 bridge node**, and **independent ROS 2 follower node**.
The ROS 2 bridge nodes run *in one Python process*, and the five follower
nodes run *in another Python process*, to simplify Windows startup.
This is **not** centralized robot coordination: no robot controller reads
another robot's messages, identity, world pose, or stored path.

`config/robots.json` controls individual forward speed, base angular speed,
steering gain, and maximum angular velocity. Edit it **before launching**
`follow_all.py`; restart the followers to load changes.

## Topics published for each robot Ri (i = 1,...,5)

| ROS 2 topic | Message | Meaning |
|:--|:--|:--|
| `/Ri/cmd_vel` | `geometry_msgs/msg/Twist` | Its own velocity command |
| `/Ri/target_visible` | `std_msgs/msg/Bool` | Visible target in current camera view |
| `/Ri/target` | `geometry_msgs/msg/Vector3Stamped` | `vector.x` distance (m), `vector.y` left-positive bearing (rad); sent only when target visible |
| `/Ri/visible_count` | `std_msgs/msg/Int32` | Count of camera-visible robot candidates |
| `/Ri/follow_mode` | `std_msgs/msg/String` | `FOLLOW` or `BASE_CURVE` |

Examples for **optional third ROS 2 terminal**:

```bat
ros2 node list
ros2 topic echo /R2/follow_mode
ros2 topic echo /R3/target_visible
ros2 topic echo /R4/target
```

To test one robot without changing the world, both scripts accept a subset:

```bat
python ros2_bridge\bridge_all.py --robots R1
python ros2_controller\follow_all.py --robots R1
```

With only R1 bridged and controlled, R2–R5 receive no wheel commands and
remain stationary. For normal Step 5, run **without** `--robots` in both
terminals. **Never** run a separate manual `/Ri/cmd_vel` publisher at the
same time as the autonomous follower for that robot.

## Reactive control rule (unchanged from Step 4)

- Constant linear velocity `v = forward_speed`, regardless of sensing state.
- If a target is visible and the measurement is fresh:
  `w = clamp(steering_gain * bearing, -max_angular_speed, +max_angular_speed)`.
- Otherwise: `w = base_angular_speed`.
- Local camera sensing selects the **nearest currently visible** model
  `simple_diff_robot` within 4 m and ±50 degrees, with Webots occlusion enabled.
- Never uses GPS, a Supervisor API, global coordinates, previous target
  positions, or target tracking. No central controller makes decisions.
- The R1 cyan FOV geometry remains a **debug overlay**; it does not mean a
  robot is actually visible when a wall obstructs the camera.

### Expected observations

- All five robots begin moving on their own curved trajectories.
- Different robots may initially turn clockwise or counterclockwise.
- When a nearby robot becomes visible, the observer turns toward it.
- Robot chains or groups *may* form, but are not guaranteed with these
  parameters/starting positions. Following can break when views are occluded.
- They may hit the arena walls or other robots: **collision avoidance is not
  implemented yet**. Short runs and frequent Webots resets are sensible.

## Check the setup without Webots/ROS 2

```bat
cd /d C:\dev\reactive_following_step5
python -m unittest discover -s tests -v
```

Pure Python tests check the proportional control law, local target selection,
world configuration, and distinct robot parameters.

## Project directory

```text
reactive_following_step5/
├── worlds/reactive_following.wbt
├── protos/SimpleDiffRobot.proto
├── controllers/udp_diff_driver/
│   ├── udp_diff_driver.py
│   └── perception.py
├── config/robots.json
├── ros2_bridge/bridge_all.py
├── ros2_controller/
│   ├── follow_all.py
│   └── follow_logic.py
├── tests/
│   ├── test_follow_logic.py
│   └── test_multi_robot.py
└── README.md
```

**Known limitations:** Webots recognition returns simulated relative object
metadata instead of processing RGB images. Confirm this sensing method is
accepted by the lecturer. No sensor noise model, distance control, collision
avoidance, trajectory logging or experiments are included at this stage.
