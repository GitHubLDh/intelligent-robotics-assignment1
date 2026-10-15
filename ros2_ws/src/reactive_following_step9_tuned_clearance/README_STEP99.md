# Step 9.9 — Closer, softer obstacle avoidance

This tuning addresses the Webots observation that R2 escapes corners but stops too far from the walls and turns too slowly. Derived from Step 9.8; the camera classifier, Webots robot, LiDAR mount and UDP bridge are unchanged.

## Control changes

- Early stop/turn zone reduced from 1.20m to 0.88m (requires a nearby diagonal obstruction too).
- Pocket/corner preemption now requires closer obstacles (~0.72m rather than ~0.95m).
- LiDAR slowdown starts at 0.73m; front hard avoidance starts at 0.50m. These are LiDAR-origin distances, not free space outside the chassis.
- Normal obstacle turning uses 0.40rad/s (23°/s), including moving clearance arcs, so robots turn less sharply while preserving the deterministic right-hand escape direction.
- The blended return to each robot's original base curvature remains at 0.77–1.10m, while the lower LiDAR gates allow a closer approach before avoidance takes over.
- Emergency thresholds (0.28m) and reverse protections are unchanged. No global pose, remembered target or heading, or supervisor data is used.

## Test

1. Fully stop previous bridge/controller processes. Open `worlds/diagnostic_head_on.wbt` and press Play.
2. Run `cd /d C:\dev\reactive_following_step9_tuned_clearance` then `python ros2_bridge\bridge_all.py --robots R1 R2`.
3. In another terminal run `python ros2_controller\follow_improved.py --robots R1 R2` from the same directory.
4. Observe R2 at a wall: it should approach closer before turning, rotate more quickly, and complete the corner without contact. Repeat a few resets.
5. Monitor `/R2/contact` and `/R2/follow_mode`; if telemetry stops, check `/R2/bridge_health` and `/R2/sim_time`.

The included Python unit tests cover control decisions and idealized corner geometry, not Webots physics. Tune conservatively based on observed minimum clearance and collision bumper results.
