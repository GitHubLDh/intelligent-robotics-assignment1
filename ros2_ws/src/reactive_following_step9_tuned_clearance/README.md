**Latest controller revision:** see [README_STEP98.md](README_STEP98.md) for the Step 9.8 corner-oscillation fix and tests.

**Step 9.7 update:** See `README_STEP97.md` for the new unified avoidance steering, robust Ctrl+C handling, bridge-health topic, and two-robot validation procedure.

**Step 9.6 patch:** see `README_STEP96.md` for the updated avoidance control and sensor health topic.

# Step 9.5 — Directional front/rear recognition (LATEST)

**Start with `README_DIRECTIONAL.md`.** This release adds smaller, visually
one-sided markers and a camera-relative body/marker classifier. The previous
README below describes earlier stages and may contain outdated camera-marker
semantics. Do not use prior A/B/C occlusion comparison scenes; recognition
occlusion remains enabled at level 2 in all included robots.

---

# Step 9.4 — Camera visibility diagnostics + prefer turning at walls

This release fixes excessive reversing near walls and adds camera-recognition
telemetry. It is derived from Step 9.3; the 0.155 m LiDAR scan-plane height,
19 cm low-wall sensing, camera FOV/range, bumper, wheel parameters, and
existing ROS 2 topics are retained. No GPS, global pose, Supervisor, target
memory, or robot-to-robot communication is used.

## Why the camera can appear not to recognize a robot

`/R1/target_visible` is **not a generic "robot seen" topic**. It is true
only when a camera sees a GREEN REAR marker eligible for following. If the
camera sees a RED FRONT marker, `target_visible` stays false by design.
A side-on robot can also have neither marker visible.

New diagnostics for each Ri:
- `/Ri/robot_visible`: true if red OR green marker seen.
- `/Ri/visible_count`: number of currently recognized red/green markers.
- `/Ri/camera_raw_count`: all current recognition objects from the camera.
- `/Ri/camera_models`: raw model types visible in the *current* camera frame.
- `/Ri/target_visible`: GREEN rear marker only (unchanged).
- `/Ri/front_marker`: RED front marker measurements (unchanged).

If raw_count is positive but visible_count is zero, the camera sees something
but not an eligible marker. This reveals sensor/matching/occlusion problems
without using privileged information. Markers are interpreted only by type,
never by robot identity. None of the diagnostic fields is used for steering.

## Changes in obstacle response

- With a near wall in front, command a safe clockwise pivot **first**, rather
  than straight reverse, provided LiDAR shows room for the footprint.
- Reserve reversing for a physically constrained pivot with clear rear AND
  rear-diagonal LiDAR sectors.
- On a bumper contact, back off at 0.10 m/s for 0.5 seconds, then wait a random
  0–1 seconds before resuming. If contact remains active, repeat recovery.
- Keep the tested head-on, corner, follow/hold priorities otherwise intact.
- Emergency contact-range LiDAR readings still result in reverse if necessary
  and safe, or a full stop if no motion is safe.

## Test 1: camera in a stationary world (DO NOT start follow_improved.py)

Open `worlds/diagnostic_camera_rear_marker.wbt`; press Play.
Run in Terminal 1 (Pixi+ROS 2 configured):

```bat
cd /d C:\dev\reactive_following_step9_camera_turn_fixed
python ros2_bridge\bridge_all.py --robots R1
```

In Terminal 2 (also ROS 2 configured):

```bat
ros2 topic echo /R1/robot_visible
```

In Terminal 3 inspect either `/R1/target_visible`, `/R1/camera_raw_count`,
`/R1/camera_models`, or `/R1/visible_count` (one command per terminal).
Expect `robot_visible=true`, `target_visible=true`, and `camera_models`
containing `rear_marker` if R4 is visible and not occluded by scenery.

Then open `worlds/diagnostic_camera_front_marker.wbt`, restart the bridge,
and verify `robot_visible=true`, `target_visible=false`, and `camera_models`
including `front_marker`. If the raw count is 0, inspect camera image via
Webots camera overlay and verify R4 is in frame; if nonzero with wrong
models, send the model names. The real-world output is decisive.

## Test 2: autonomous wall avoidance

Open `worlds/diagnostic_lidar_low_wall_close.wbt`, press Play, bridge R1
as above, then launch a second terminal:

```bat
python ros2_controller\follow_improved.py --robots R1
```

A wall at 0.4m should cause `AVOID_FRONT` or `CLEAR_FORWARD_TURN`, with
zero forward and a consistent yaw, not an endless back-and-forth reverse.
Observe `/R1/follow_mode` and `/R1/contact_events` separately.

## Test 3: head-on and follow

Run `worlds/diagnostic_head_on.wbt` with bridge/controllers for R1 R2.
Then test `worlds/diagnostic_r1_r4_stationary.wbt` with only R1 bridge +
follower. These confirm head-on avoidance and FOLLOW_HOLD still work.

## Regression tests

```bat
python -m unittest discover -s tests -v
```

Local tests do not validate Webots camera occlusion or physics. Complete
camera and wall verification before running all five robots.

---

# Step 9.3 — LiDAR self-echo + corner-turn consistency

## Why this patch

In the Step 9.2 low-wall diagnostic, `FRONT=0.40m` confirmed wall
visibility, but `REAR=0.12m` even though the world contains no rear
obstacle. This value is consistent with a return from the robot's own green
rear marker (~0.12–0.13m behind the LiDAR). This is a geometry-based
hypothesis until checked in Webots.

The head-on diagnostic succeeded for R1, but R2 rocked left/right in an
arena corner. The Step 9.2 code could reverse yaw preference when flank
measurements changed marginally, and a false blocked rear prevented escape.

## Changes

- Raise the local LiDAR mount from z=0.050 to **z=0.070**. For an
  upright robot starting at z=0.085m, the scan plane is z=0.155m:
  above the robot's rear marker at close range but **below 0.19m** walls.
- Preserve all three LiDAR layers, the camera and green/red markers,
  and the existing ROS 2 topics and communication protocol.
- Use a fixed clockwise corner turn when the current scan permits a
  safe pivot. If too confined, back off only when current rear/diagonal
  clearances permit; otherwise stop. A small flank-distance fluctuation
  cannot reverse the escape steering direction.
- Tighten the forward-clearance transition from the corner, without
  modifying camera-following or head-on steering rules.
- No target memory, global location, Supervisor, GPS, past-scan memory or
  fixed leader is used.

## Test 1 — Empty-floor LiDAR calibration (mandatory)

Open `worlds/diagnostic_lidar_clear_space.wbt`. Start Webots in Play mode.
In two configured ROS 2 Windows terminals:

```
cd /d C:\dev\reactive_following_step9_self_echo_fixed
python ros2_bridge\bridge_all.py --robots R1
```

```
cd /d C:\dev\reactive_following_step9_self_echo_fixed
python ros2_bridge\inspect_lidar.py --robot R1
```

Expected `FRONT` **and** `REAR` approximately 2.50m, assuming the
robot is upright and the scanner is not seeing its own chassis/markers.
If `REAR` remains around 0.12m, stop and report; do not mask the near
readings as free space, as that would hide real rear obstacles.

## Test 2 — Low 19cm wall still visible

Open `worlds/diagnostic_lidar_low_wall_close.wbt` and restart the same
bridge/inspector. Expected FRONT approx 0.40m, REAR approx 2.50m.

## Test 3 — R2 post-head-on corner escape

Open `worlds/diagnostic_head_on.wbt`, reset the simulation, press Play.
In terminal 1:

```
cd /d C:\dev\reactive_following_step9_self_echo_fixed
python ros2_bridge\bridge_all.py --robots R1 R2
```

In terminal 2:

```
cd /d C:\dev\reactive_following_step9_self_echo_fixed
python ros2_controller\follow_improved.py --robots R1 R2
```

In optional monitor terminals:

```
python ros2_bridge\inspect_lidar.py --robot R2
ros2 topic echo /R2/follow_mode
```

After the head-on encounter, R2 should clear the wall/corner, **not**
rock in place. Watch for `CORNER_CLEAR_TURN`, `CORNER_BACKOFF`, and
`CLEAR_FORWARD_TURN`. If stuck, report R2's six sector ranges and repeated
modes. Stop if the bumper stays active or the robot remains stuck 10–15s.

Only after these three tests pass, repeat the isolated stationary-target
and moving-chain tests, then the full five-robot arena.

## Regression tests

```
python -m unittest discover -s tests -v
```

Local tests cannot confirm Webots physics or whether the sensor return
actually comes from the robot's own marker. That requires the empty-floor
calibration.

---

# Step 9.2 — Low-wall LiDAR blind-spot fix

## Why this update exists

In the Step 9.1 head-on test, R1 and R2 avoided one another, but R2 then
collided with an internal wall and remained stuck. An important geometric
problem was found: the LiDAR origin was about **0.225 m above ground**, while
internal walls stand only **0.19 m**. With its ±0.08 rad vertical layers, the
lowest ray may pass *above* a wall at close range (below roughly 0.44 m).
This is a plausible explanation; the cause has **not** yet been confirmed in
Webots with the actual R2 collision.

### Changes

- `SimpleDiffRobot.proto`: lower LiDAR origin to **0.135 m** above the floor
  (local `translation 0 0 0.050`).
- Narrow LiDAR vertical field of view from 0.16 to **0.08 rad** (three layers),
  keeping the lower layer above the floor and chassis top across the rated
  2.5 m range in the level-pose approximation.
- New world `worlds/diagnostic_lidar_low_wall_close.wbt` with a **0.19 m** tall
  wall at approximately **0.40 m** LiDAR distance. The original high-mounted
  sensor could miss this wall.
- **No changes** to ROS 2 bridge, LiDAR processing, marker recognition,
  following, head-on behavior, wheel motors, collision physics or corner logic.
- Added geometry unit tests; the sensor model must still be tested in Webots.

## Test A — Low wall (no autonomous controller)

Extract to `C:\dev\reactive_following_step9_low_wall_fixed`. Stop old bridge
and follower terminals and close the previous Webots world.

1. Open `worlds/diagnostic_lidar_low_wall_close.wbt` and press Play.
2. In ROS 2 Terminal 1:

   ```bat
   cd /d C:\dev\reactive_following_step9_low_wall_fixed
   python ros2_bridge\bridge_all.py --robots R1
   ```
3. In ROS 2 Terminal 2:

   ```bat
   cd /d C:\dev\reactive_following_step9_low_wall_fixed
   python ros2_bridge\inspect_lidar.py --robot R1
   ```
4. `FRONT` should detect about **0.40 m** (allow tolerance). `REAR` should be
   near 2.5 m. **Do not start autonomous control** for this first diagnostic.
   If FRONT remains 2.50 m, provide the full six distance readings and a
   screenshot of the LiDAR rays.

## Test B — Head-on followed by wall avoidance

Only after Test A passes:

1. Open `worlds/diagnostic_head_on.wbt`, reset, Play.
2. Bridge: `python ros2_bridge\bridge_all.py --robots R1 R2`.
3. Follower: `python ros2_controller\follow_improved.py --robots R1 R2`.
4. Watch both robots **after** they pass. R2 should detect the internal wall,
   steer away or safely escape, and continue moving without prolonged contact.
5. To inspect R2, use `ros2 topic echo /R2/follow_mode` and
   `python ros2_bridge\inspect_lidar.py --robot R2` in extra terminals.

Report whether R2 is free after 30 seconds, whether any bumper events were
observed, and the mode if it gets stuck. Do not run five robots until this test
passes. This is an *unverified* physics fix, not a promise that all collisions
are solved.

## Previous Step 9.1 documentation

# Step 9.1 — Corner Oscillation Fix (Camera + 360° LiDAR)

This is a targeted patch to the Step 9 project based on R2 turning left and
right repeatedly at the corner. The camera, 360° LiDAR, ROS 2 bridge, robot
PROTO/physics, separate wheel controllers, and worlds are unchanged.

## Changes made

- New `CORNER_CLEAR_TURN`: in an obstructed corner, turn consistently to
  the right when there is space; don't reverse turning direction from tiny
  left-versus-right LiDAR fluctuations.
- New `CLEAR_FORWARD_TURN`: prevents `BASE_CURVE_SLOW` from undoing a
  collision-avoidance turn while the front path is still obstructed.
- New `CORNER_BACKOFF`: if rotating would put the chassis too close to a side
  wall, reverse straight ONLY when rear and rear-diagonal clearances permit.
- New `CORNER_BLOCKED`: stop if neither a safe pivot nor reverse is available;
  the controller cannot guarantee an escape from every completely trapped
  configuration without additional planning/memory.
- Existing close-range emergency braking, head-on detection and stationary
  target `FOLLOW_HOLD` rules remain ahead of the corner logic.

The choice is based on each current LiDAR scan. No remembered escape heading,
map, GPS, target, or previous scan is used. A fixed right preference does not
by itself guarantee collision-free motion in arbitrary obstacle layouts.

## First test: R2 corner (only R2)

1. Unzip into `C:\dev`, resulting in
   `C:\dev\reactive_following_step9_corner_fixed`.
2. Stop previous bridge/controller terminals. Open in Webots:
   `C:\dev\reactive_following_step9_corner_fixed\worlds\diagnostic_r2_corner.wbt`
   and press Play.
3. ROS 2 Developer Command Prompt (with Pixi + lyrical setup), terminal 1:

   ```bat
   cd /d C:\dev\reactive_following_step9_corner_fixed
   python ros2_bridge\bridge_all.py --robots R2
   ```
4. Terminal 2:

   ```bat
   cd /d C:\dev\reactive_following_step9_corner_fixed
   python ros2_controller\follow_improved.py --robots R2
   ```
5. Optional terminal 3:

   ```bat
   ros2 topic echo /R2/follow_mode
   ```

Expected: R2 turns persistently away from the corner, finds a clear forward
sector, and then drives out. An initial brief right-hand pivot is normal;
repeating left/right pivots at the same spot is **not** a pass. Stop if it
contacts a wall or remains blocked for more than 15 seconds.

## Next regressions (only after R2 corner passes)

- `diagnostic_lidar_front_wall.wbt` with R1.
- `diagnostic_head_on.wbt` with R1 and R2.
- `diagnostic_r1_r4_stationary.wbt` with only R1 bridged/controlled.
- `diagnostic_chain_three.wbt` with R1, R2, R3.

Do not proceed to the five-robot trial until the isolated wall/head-on tests
pass. The added corner policy and its thresholds require real Webots physics
validation. This ZIP has not been run inside Webots here.

## Regression tests

```bat
cd /d C:\dev\reactive_following_step9_corner_fixed
python -m unittest discover -s tests -v
```

The new tests include an **approximate kinematic** northeastern-corner model
that checks for steering consistency and forward progress. It does not simulate
actual robot friction, bumper contact, or LiDAR reflections.

---

# Step 9: Five-robot following with Camera + 360° LiDAR

This project replaces the six narrow ultrasonic sensors from **Step 8** with one
**360° LiDAR** per robot. The working Step 8 robot camera, front/rear markers,
independent robot parameters, differential-drive motors, bumper, 2 passive
supports, and validated three-robot chain are retained.

## Architecture

- **Webots:** each `SimpleDiffRobot` has a forward camera (100°, 4 m),
  a `safety_lidar` (360 rays × 3 vertical layers, 2.5 m), two wheels and an
  evaluation-only contact bumper. The LiDAR sits above the chassis, with a
  slightly downward-looking layer for low walls and neighboring robots.
- **Webots controller:** merges the three LiDAR layers conservatively into a
  single 360-ray scan and transmits the *current* camera and LiDAR readings over
  the existing per-robot local UDP link (`15001`–`15005`).
- **ROS 2 bridge:** publishes `/<robot>/scan` (`sensor_msgs/LaserScan`) and
  `/<robot>/local_sensors` (`std_msgs/Float64MultiArray` of length 364), plus
  camera topics, contact readings and sim time. It also forwards `cmd_vel`.
- **ROS 2 follower:** reads only its own robot's simultaneous local snapshot.
  It applies full-direction collision protection before following a green rear
  marker, or passing a red front marker. It never uses a Supervisor, GPS,
  another robot's topics, identities or previous target readings.

**A camera-observed rear marker can be filtered from the collision scan only if
its current bearing and range match a LiDAR return.** A closer wall/obstacle
stays in the scan; raw emergency clearance is never masked. This filtering is
approximate and requires validation in Webots.

## Install and start on Windows

Extract the ZIP under `C:\dev` so the root is:

`C:\dev\reactive_following_step9_lidar`

Use your **configured ROS 2 Developer Command Prompt**, with `pixi` activated
and the `C:\dev\lyrical\install\local_setup.bat` setup sourced. Close previous
Step 8 bridge/controller terminals first.

### Mandatory Test 1 — LiDAR front/rear calibration (no autonomous motion)

1. In Webots open:
   `C:\dev\reactive_following_step9_lidar\worlds\diagnostic_lidar_front_wall.wbt`
   and press Play. R1 is stationary and facing an orange wall at +X.
2. ROS 2 terminal 1:

   ```bat
   cd /d C:\dev\reactive_following_step9_lidar
   python ros2_bridge\bridge_all.py --robots R1
   ```
3. ROS 2 terminal 2:

   ```bat
   cd /d C:\dev\reactive_following_step9_lidar
   python ros2_bridge\inspect_lidar.py --robot R1
   ```
4. Expected: `FRONT` is about **0.7–0.8 m**, and `REAR` is near **2.5 m**.
   These are approximate and depend on rendered geometry.
5. Stop both terminals and open
   `worlds\diagnostic_lidar_rear_wall.wbt`; restart the same bridge+inspector.
   Expected: `REAR` is about **0.7–0.8 m**, while `FRONT` is near **2.5 m**.

**Do not activate autonomous control if front/rear appear swapped, scans stay
at 2.5 m in front of the wall, or the scan topic is absent.** Report the
inspector output first, as scan orientation and obstacle visibility have not
been physically verified on your Webots installation.

The standard scan is also available with:

```bat
ros2 topic echo /R1/scan
```

This is verbose (360 samples), so prefer the inspector.

### Test 2 — Autonomous wall avoidance

Reset `diagnostic_lidar_front_wall.wbt` and keep its R1 bridge running.
In a **third** ROS 2 terminal run:

```bat
cd /d C:\dev\reactive_following_step9_lidar
python ros2_controller\follow_improved.py --robots R1
```

R1 should slow and turn/reverse **before** touching the wall. It should never
run into the wall at normal BASE_CURVE speed. Stop if collision is observed.
You can monitor modes with:

```bat
ros2 topic echo /R1/follow_mode
```

### Test 3 — Safety and moving chain regressions

After stopping all previous programs, test in turn:

- `worlds\diagnostic_r2_corner.wbt` using `--robots R2` for bridge and follower.
- `worlds\diagnostic_head_on.wbt` using `--robots R1 R2` for both programs.
- `worlds\diagnostic_r1_r4_stationary.wbt` using `--robots R1` for both:
  R1 should approach R4 and remain stopped in `FOLLOW_HOLD`.
- `worlds\diagnostic_chain_three.wbt` using `--robots R1 R2 R3` for both:
  aim for a moving three-robot chain for >=10 seconds.

### Test 4 — All five robots

Open `worlds\reactive_following_trial1.wbt`, reset and play.

Terminal 1:

```bat
cd /d C:\dev\reactive_following_step9_lidar
python ros2_bridge\bridge_all.py
```

Terminal 2:

```bat
cd /d C:\dev\reactive_following_step9_lidar
python ros2_controller\follow_improved.py
```

Observe for 60 seconds. Repeat with `reactive_following_trial2.wbt` and
`reactive_following_trial3.wbt` if the first run is stable. Measure bumper
contact duration, stalled time, and maximum moving chain length. Contact
**episodes** alone can hide long periods of collision.

## Topics (per robot, e.g. R1)

| Topic | Type | Purpose |
|---|---|---|
| `/R1/scan` | `sensor_msgs/LaserScan` | 360° current distance scan (m) |
| `/R1/local_sensors` | `std_msgs/Float64MultiArray` | Atomic camera + 360-range snapshot |
| `/R1/target` | `geometry_msgs/Vector3Stamped` | Green marker distance/bearing, if visible |
| `/R1/front_marker` | `geometry_msgs/Vector3Stamped` | Red marker distance/bearing (NaN if absent) |
| `/R1/target_visible` | `std_msgs/Bool` | Green marker detected now |
| `/R1/follow_mode` | `std_msgs/String` | Current driving mode |
| `/R1/contact`, `/R1/contact_events` | `Bool`, `Int32` | Evaluation-only bumper telemetry |
| `/R1/cmd_vel` | `geometry_msgs/Twist` | Robot wheel-speed request |

LiDAR scan indices (confirmed by **software logic only**, must calibrate in Webots):
`0 = behind`, `90 = right`, `180 = front`, `270 = left`. Angles in the
ROS `LaserScan` header run from `-π` to approximately `+π`.

## Initial tuning values

- Base cruise speed: individual robots 0.12–0.16 m/s.
- Maximum following speed: **0.21 m/s**.
- Desired camera-measured stand-off: **0.55 m** with **0.06 m** hold band.
- LiDAR slows motion inside **1.00 m**.
- Frontal avoidance inside **0.58 m**; emergency inside **0.28 m**.
- Hard flank clearance **0.19 m** and rear reversing only if rear/diagonal
  beams are clear (`rear > 0.55 m`). These are provisional distances measured
  from LiDAR origin, not chassis edge.

Edit `config\robots.json` to tune per robot. Do not set clearance thresholds
smaller until Webots shows no wheel/bumper contacts at those values.

## Local code tests

```bat
cd /d C:\dev\reactive_following_step9_lidar
python -m unittest discover -s tests -v
```

These tests validate packet encoding, 3-layer combination, scan directions,
collision priorities and following states, but **cannot** establish Webots
LiDAR geometry, physical contact, or reliable chain formation. Step 8's
original project remains an independent comparison baseline.

## Assignment restrictions

Only current local sensor data, plus permitted motor dynamics and PI/PID
internal state. No GPS, global coordinates, map building, Supervisor API,
tracking/memory of previously followed robots, or inter-robot coordination.
Front/rear marker **types**, not robot identities, are recognized by the camera.
If a target disappears, the controller does not extrapolate its position.

## Important limitations

- A LiDAR doesn't guarantee collision-free behavior; LiDAR height and vertical
  scan layers must detect low wall surfaces and robot bodies.
- The front/rear convention must be **confirmed in Test 1** before any driving.
- Target masking is based on relative bearing/range and may be too permissive
  in overlapping robot-and-wall situations. The nearest critical raw beam
  remains a fail-safe, but test the crowded scenarios carefully.
- The original 6 sonar sensors were **replaced**, not run alongside LiDAR.
- Windows UDP bridge still stops if data/commands time out.

Step 9.9 clearance tuning: see `README_STEP99.md`.
