"""Independent reactive followers R1-R5 in a ROS 2 node/process.

Each robot has its own Node, subscriptions, state and configuration, using
only its front camera's current nearest-visible-robot distance and bearing.
There is no robot-to-robot messaging, GPS, map, target history or coordination.
"""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from reactive_following.follow_logic import ControlParameters, command_for_detection
except ImportError:
    from follow_logic import ControlParameters, command_for_detection

SENSOR_TIMEOUT_S = 0.45


def get_default_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('reactive_following'))
        cfg = share / 'config' / 'robots.json'
        if cfg.is_file():
            return cfg
    except Exception:
        pass
    local = Path(__file__).resolve().parent.parent / "config" / "robots.json"
    if local.is_file():
        return local
    return Path.cwd() / "config" / "robots.json"


def load_config(path=None):
    if path is None:
        path = get_default_config_path()
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    names = {f"R{i}" for i in range(1, 6)}
    if set(data) != names:
        raise ValueError(f"Configuration must contain exactly {sorted(names)}")
    result = {}
    for name, values in data.items():
        cfg = ControlParameters(**values)
        if not (0 < cfg.forward_speed <= 0.25 and
                -cfg.max_angular_speed <= cfg.base_angular_speed <= cfg.max_angular_speed and
                0 <= cfg.steering_gain <= 10 and
                0 < cfg.max_angular_speed <= 2.0 and
                all(math.isfinite(v) for v in vars(cfg).values())):
            raise ValueError(f"Invalid controller settings for {name}: {values}")
        result[name] = cfg
    return result


class ReactiveFollower(Node):
    def __init__(self, robot_name, cfg):
        super().__init__(f"{robot_name.lower()}_reactive_follower")
        self.robot_name = robot_name
        self.cfg = cfg
        self.visible = False
        self.bearing_rad = None
        self.distance_m = None
        self.last_visible_time = None
        self.last_target_time = None
        self.last_mode = None
        self.last_status_at = 0.0
        ns = f"/{robot_name}"

        self.create_subscription(Bool, f"{ns}/target_visible", self.on_visible, 10)
        self.create_subscription(Vector3Stamped, f"{ns}/target", self.on_target, 10)
        self.velocity_pub = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, f"{ns}/follow_mode", 10)
        self.create_timer(0.1, self.on_tick)
        self.get_logger().info(
            f"{robot_name}: v={cfg.forward_speed:.2f} m/s, "
            f"base w={cfg.base_angular_speed:+.2f} rad/s, Kp={cfg.steering_gain:.2f}")

    def on_visible(self, msg):
        self.visible = bool(msg.data)
        self.last_visible_time = time.monotonic()
        if not self.visible:
            self.bearing_rad = None
            self.distance_m = None
            self.last_target_time = None

    def on_target(self, msg):
        if msg.header.frame_id != f"{self.robot_name}/front_camera":
            return
        self.bearing_rad = float(msg.vector.y)
        self.distance_m = float(msg.vector.x)
        self.last_target_time = time.monotonic()

    def on_tick(self):
        now = time.monotonic()
        fresh_visible = (self.visible and self.last_visible_time is not None
                         and now - self.last_visible_time < SENSOR_TIMEOUT_S)
        fresh_target = (self.last_target_time is not None
                        and now - self.last_target_time < SENSOR_TIMEOUT_S)
        can_follow = fresh_visible and fresh_target and self.distance_m is not None
        v, w, mode = command_for_detection(
            can_follow,
            self.bearing_rad if can_follow else None,
            self.cfg,
            distance_m=self.distance_m if can_follow else None,
        )
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.velocity_pub.publish(msg)
        self.mode_pub.publish(String(data=mode))
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(f"{self.robot_name} mode: {mode}")
        if mode == "FOLLOW" and now - self.last_status_at >= 3.0:
            self.last_status_at = now
            self.get_logger().info(
                f"{self.robot_name} target range={self.distance_m:.2f} m, "
                f"bearing={math.degrees(self.bearing_rad):+.1f} deg, "
                f"turn={w:+.2f} rad/s, speed={v:.2f} m/s")

    def stop(self):
        self.velocity_pub.publish(Twist())


def main(args=None):
    import sys
    from rclpy.utilities import remove_ros_args

    clean_args = remove_ros_args(args=sys.argv[1:] if args is None else args)
    parser = argparse.ArgumentParser(description="Run independent R1-R5 reactive followers")
    parser.add_argument("--robots", nargs="+", choices=[f"R{i}" for i in range(1, 6)],
                        default=[f"R{i}" for i in range(1, 6)],
                        help="Subset to start, e.g. --robots R1 R2 (default: all)")
    parsed, _ = parser.parse_known_args(clean_args)
    configs = load_config()

    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for name in dict.fromkeys(parsed.robots):
            node = ReactiveFollower(name, configs[name])
            nodes.append(node)
            executor.add_node(node)
        print("Autonomous followers running: " + ", ".join(n.robot_name for n in nodes), flush=True)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
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

