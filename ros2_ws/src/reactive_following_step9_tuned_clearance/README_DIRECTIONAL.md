# Step 9.5 — Direction-aware camera markers and target classification

This is a full project based on the previously working Step 9 LiDAR+camera version.
The obstacle-avoidance controller (`safe_logic.py`), LiDAR, camera FOV/range,
robot physics and wheel motor parameters are preserved. Recognition occlusion
remains **2**; no world uses disabled occlusion.

## Fix

The original markers were broad, double-sided boxes. Webots could report both
`front_marker` and `rear_marker` from the same robot, even when only its front
was presented to the camera. Presence of a green marker alone was NOT enough
for target selection, and could trigger head-on pursuit.

**Visual geometry:** thin (2 mm) red/green panels in front of a larger opaque
black backing, entirely within the original collision envelope. These are
only visually exposed on the corresponding front/rear side. In addition,
Webots Recognition may still return both marker TYPES via their recognition
volumes: the geometry alone cannot guarantee exclusive recognized objects.

**Classifier:** associates each marker with a `simple_diff_robot` body from
*the current camera frame* by camera-relative position (no robot IDs). It
compares projected front/rear marker distance relative to the associated body.
Whichever surface is closer to the camera and sufficiently offset determines
`FRONT` or `REAR`. A side view, ambiguous multiple body association, or a
marker with no matched body is rejected. Raw `camera_models` can still list
both markers; inspect **`/R1/camera_classification`** and `/R1/target_visible`
for the controller's accepted interpretation.

No GPS/global localization, Supervisor, target memory, recognition IDs, or
orientation metadata are used. LiDAR still prevents close-range collisions.

## Test A — head-on (reject target)

Close old Webots world, bridge and follower. Unzip to `C:\dev`.
Open `worlds/diagnostic_direction_front.wbt`, press Play.

Terminal 1:

```
cd /d C:\dev\reactive_following_step9_directional_markers
python ros2_bridge\bridge_all.py --robots R1
```

Terminal 2: run separately, Ctrl+C between them:

```
ros2 topic echo /R1/camera_classification
ros2 topic echo /R1/robot_visible
ros2 topic echo /R1/target_visible
ros2 topic echo /R1/camera_models
```

Expected: `camera_classification` includes `front=1 rear=0`; `robot_visible`
true; `target_visible` false. `camera_models` may still list BOTH marker types;
that is raw recognition, not accepted orientation.

## Test B — rear-facing target (accept following)

Open `worlds/diagnostic_direction_rear.wbt`, press Play, restart bridge.
Expected: `front=0 rear=1`, `robot_visible=true`, `target_visible=true`.

## Test C — multiple robots and live movement

After A+B, open `worlds/diagnostic_camera_front_marker.wbt` (the earlier arena
scenario). Restart bridge with `--robots R1` and verify it rejects front
marker from R4 even when `camera_raw_count` is 3 or more.
Then retest `worlds/diagnostic_head_on.wbt` with bridges/controllers R1 R2,
followed by `worlds/diagnostic_chain_three.wbt` and five-robot trials.

**Note:** This package passes offline Python tests only. Webots Recognition
and occlusion must still be confirmed in your actual simulation. If
`camera_classification` is ambiguous or `target_visible` wrong, send both
`/R1/camera_classification` and `/R1/camera_raw_count` with a Webots camera
screenshot; do not resume five-robot experiments until these tests pass.
