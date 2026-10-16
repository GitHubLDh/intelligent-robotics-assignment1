"""Five independent reactive followers, one ROS 2 node per robot.

Implements Assignment 1 slide 4 via reactive_logic.follow_command. Each
controller reads only its own robot's frontal camera and LiDAR; nothing is
published or subscribed between robots.

Reports the four metrics slide 7 asks for - following duration, angular error,
target losses and collisions. The contact bumper is read for the collision
COUNT only and never influences behaviour, since it is not one of the allowed
frontal sensors.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray
from reactive_logic import FollowParameters, clamp, follow_command
from sensor_snapshot import parse_snapshot

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "robots.json"
FRESH_S = 0.25
ROBOT_NAMES = [f"R{i}" for i in range(1, 6)]


def load_config(path=CONFIG_PATH, overrides=None):
    """Build per-robot parameters, ignoring keys this controller does not use.

    robots.json still carries settings for the previous controller; only the
    fields FollowParameters declares are read, so the file needs no edit.
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if set(data) != set(ROBOT_NAMES):
        raise ValueError("Expected all five robot configs")
    # steering_gain in robots.json was tuned for the old control law and
    # saturates this one over most of the camera cone; take it from
    # FollowParameters (or --steering-gain) instead. Per-robot
    # base_angular_speed and forward_speed are still honoured, so the robots
    # keep their individual base curves.
    allowed = set(FollowParameters.__dataclass_fields__) - {"steering_gain"}
    result = {}
    for name, settings in data.items():
        kwargs = {k: v for k, v in settings.items() if k in allowed}
        kwargs.update(overrides or {})
        cfg = FollowParameters(**kwargs)
        if not (0 < cfg.forward_speed <= 0.25 and 0 <= cfg.steering_gain <= 10
                and 0 < cfg.max_angular_speed <= 2
                and abs(cfg.base_angular_speed) <= cfg.max_angular_speed
                and all(math.isfinite(x) for x in vars(cfg).values()
                        if isinstance(x, float))):
            raise ValueError(f"Invalid {name} config")
        result[name] = cfg
    return result


class ReactiveFollower(Node):
    def __init__(self, name, cfg):
        super().__init__(name.lower() + "_reactive_follower")
        self.name_ = name
        self.cfg = cfg
        self.snapshot = None
        self.snapshot_at = None
        self.last_mode = None
        self.contact_active = False
        # Slide 7 metrics.
        self.following_s = 0.0
        self.bearing_error_sum = 0.0
        self.bearing_error_n = 0
        self.target_losses = 0
        self.collisions = 0
        self.last_tick = None
        self.last_v = 0.0
        self.last_w = 0.0
        ns = "/" + name
        self.create_subscription(Float64MultiArray, ns + "/local_sensors",
                                 self.on_local_sensors, 10)
        self.create_subscription(Bool, ns + "/contact", self.on_contact, 10)
        self.cmd_pub = self.create_publisher(Twist, ns + "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, ns + "/follow_mode", 10)
        self.create_timer(0.05, self.on_tick)
        self.get_logger().info(
            f"{name}: reactive follower; v={cfg.forward_speed:.2f} "
            f"gain={cfg.steering_gain:.2f} "
            f"standoff={cfg.desired_distance if cfg.keep_distance else None} "
            f"avoid={cfg.avoid_enabled}")

    def on_local_sensors(self, msg):
        self.snapshot = parse_snapshot(msg.data)
        self.snapshot_at = time.monotonic()

    def on_contact(self, msg):
        active = bool(msg.data)
        # Counted as a metric only: never fed back into the command.
        if active and not self.contact_active:
            self.collisions += 1
        self.contact_active = active

    def on_tick(self):
        now = time.monotonic()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        stale = (self.snapshot_at is None or now - self.snapshot_at > FRESH_S
                 or self.snapshot is None)
        if stale:
            v, w, mode, bearing = 0.0, 0.0, "SENSOR_FAILSAFE", None
        else:
            distance, bearing = self.snapshot[0], self.snapshot[1]
            oncoming_d, oncoming_b = self.snapshot[2], self.snapshot[3]
            ranges = self.snapshot[-1]
            v, w, mode = follow_command(distance, bearing, ranges, self.cfg,
                                        oncoming_d, oncoming_b)

        # Acceleration limits: ramp toward the commanded values rather than
        # stepping to them, so a target appearing at the edge of the cone does
        # not produce an instant full-rate swing.
        if dt > 0.0:
            dv = self.cfg.max_linear_accel * dt
            dw = self.cfg.max_angular_accel * dt
            v = clamp(v, self.last_v - dv, self.last_v + dv)
            w = clamp(w, self.last_w - dw, self.last_w + dw)
        self.last_v, self.last_w = v, w

        if mode in ('FOLLOW', 'YIELD_TURN'):
            self.following_s += dt
            if bearing is not None:
                self.bearing_error_sum += abs(bearing)
                self.bearing_error_n += 1
        if (self.last_mode in ('FOLLOW', 'YIELD_TURN')
                and mode not in ('FOLLOW', 'YIELD_TURN')):
            self.target_losses += 1

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        self.mode_pub.publish(String(data=mode))
        if mode != self.last_mode:
            self.last_mode = mode
            extra = "" if bearing is None else f" bearing={bearing:+.3f}"
            self.get_logger().info(
                f"{self.name_} {mode} v={v:.2f} w={w:+.2f}{extra}")

    def metrics(self):
        mean_error = (self.bearing_error_sum / self.bearing_error_n
                      if self.bearing_error_n else float('nan'))
        return (f"{self.name_}: following={self.following_s:6.1f}s  "
                f"mean|bearing|={mean_error:.3f}rad  "
                f"target_losses={self.target_losses:3d}  "
                f"collisions={self.collisions:3d}")

    def stop(self):
        try:
            if rclpy.ok(context=self.context):
                self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", nargs="+", choices=ROBOT_NAMES,
                        default=ROBOT_NAMES)
    # Slide 7 compares identical initial conditions across turning strengths.
    parser.add_argument("--steering-gain", type=float,
                        help="override all robots: 0 = no following")
    parser.add_argument("--avoid", action="store_true",
                        help="enable the frontal collision-avoidance extension")
    parser.add_argument("--slow", action="store_true",
                        help="also ease off for walls using the LiDAR")
    parser.add_argument("--constant-speed", action="store_true",
                        help="spec-pure slide 4 baseline: no standoff at all")
    args = parser.parse_args()
    overrides = {}
    if args.steering_gain is not None:
        overrides["steering_gain"] = args.steering_gain
    if args.avoid:
        overrides["avoid_enabled"] = True
    if args.slow:
        overrides["slow_enabled"] = True
    if args.constant_speed:
        overrides["keep_distance"] = False
    cfg = load_config(overrides=overrides)

    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for name in dict.fromkeys(args.robots):
            node = ReactiveFollower(name, cfg[name])
            nodes.append(node)
            executor.add_node(node)
        print("Reactive followers running: " + ", ".join(args.robots), flush=True)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n--- run metrics (slide 7) ---", flush=True)
        for node in nodes:
            print(node.metrics(), flush=True)
        for node in nodes:
            node.stop()
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
