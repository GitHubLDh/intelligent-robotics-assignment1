# Step 9.9 — Faster, closer obstacle avoidance

This tuning addresses the Webots observation that R2 escapes corners but stops too far from the walls and turns too slowly. Derived from Step 9.8; the camera classifier, Webots robot, LiDAR mount and UDP bridge are unchanged.

## Control changes

- Early stop/turn zone reduced from 1.20m to 0.88m (requires a nearby diagonal obstruction too).
- Pocket/corner preemption now requires closer obstacles (~0.72m rather than ~0.95m).
- LiDAR slowdown starts at 0.82m rather than 1.00m; front hard avoidance at 0.52m rather than 0.58m. These are LiDAR-origin distances, not free space outside the chassis.
- Normal obstacle turning rate rises from 0.80rad/s (46°/s) to 1.05rad/s (60°/s). Moving clearance arcs use up to 0.075m/s at -0.85rad/s instead of 0.035m/s at -0.45rad/s.
- The blended return to each robot's original base curvature now operates at 0.77–1.10m instead of 1.10–1.55m.
- Emergency thresholds (0.28m) and reverse protections are unchanged. No global pose, remembered target or heading, or supervisor data is used.

## Test

1. Fully stop previous bridge/controller processes. Open `worlds/diagnostic_head_on.wbt` and press Play.
2. Run `cd /d C:\dev\reactive_following_step9_tuned_clearance` then `python ros2_bridge\bridge_all.py --robots R1 R2`.
3. In another terminal run `python ros2_controller\follow_improved.py --robots R1 R2` from the same directory.
4. Observe R2 at a wall: it should approach closer before turning, rotate more quickly, and complete the corner without contact. Repeat a few resets.
5. Monitor `/R2/contact` and `/R2/follow_mode`; if telemetry stops, check `/R2/bridge_health` and `/R2/sim_time`.

The included Python unit tests cover control decisions and idealized corner geometry, not Webots physics. Tune conservatively based on observed minimum clearance and collision bumper results.
