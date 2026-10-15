"""Five independent safe reactive followers, one ROS 2 node per robot.

Run instead of follow_all.py or run_trial.py. Camera/LiDAR data is local;
controllers neither publish nor subscribe to other robots' topics.
"""
import argparse
import json
import math
import random
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, Float64MultiArray
from safe_logic import (COLLISION_BACKOFF_S, SafeParameters,
                        collision_recovery_command, safe_command)
from sensor_snapshot import parse_snapshot

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "robots.json"
FRESH_S = 0.25


def load_config(path=CONFIG_PATH):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    names = [f"R{i}" for i in range(1, 6)]
    if set(data) != set(names):
        raise ValueError("Expected all five robot configs")
    result = {name: SafeParameters(**settings) for name, settings in data.items()}
    for name, cfg in result.items():
        if not (0 < cfg.forward_speed <= 0.25
                and cfg.forward_speed <= cfg.follow_max_speed <= 0.25
                and 0.3 < cfg.lidar_slow_range < 2.5
                and 0 < cfg.max_angular_speed <= 2
                and abs(cfg.base_angular_speed) <= cfg.max_angular_speed
                and 0 <= cfg.steering_gain <= 10 and cfg.distance_gain > 0
                and 0.12 < cfg.lidar_emergency < cfg.lidar_front_avoid
                and cfg.lidar_front_avoid < cfg.lidar_slow_range
                and 0.16 < cfg.lidar_side_avoid < 0.6
                and 0.35 < cfg.lidar_rear_reverse_clear < 1.5
                and 0 < cfg.desired_rear_range < 4
                and all(math.isfinite(x) for x in vars(cfg).values())):
            raise ValueError(f"Invalid {name} config")
    return result


class SafeFollower(Node):
    def __init__(self, name, cfg):
        super().__init__(name.lower()+"_safe_follower")
        self.name_ = name
        self.cfg = cfg
        # Latest complete camera + LiDAR snapshot replaces the previous one.
        # In particular, no-green in the CURRENT frame becomes None immediately;
        # an old bearing is never reused to search for a lost robot.
        self.local_snapshot = None
        self.last_snapshot_at = None
        self.last_mode = None
        self.contact_active = False
        self.collision_backoff_until = 0.0
        self.collision_restart_at = 0.0
        ns = "/"+name
        self.create_subscription(Float64MultiArray, ns+"/local_sensors",
                                 self.on_local_sensors, 10)
        self.create_subscription(Bool, ns+"/contact", self.on_contact, 10)
        self.cmd_pub = self.create_publisher(Twist, ns+"/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, ns+"/follow_mode", 10)
        # Communication diagnostic only; never used for navigation decisions.
        self.sensor_health_pub = self.create_publisher(String, ns+"/sensor_health", 10)
        self.create_timer(0.05, self.on_tick)
        self.get_logger().info(f"{name}: safe follower running; v_base={cfg.forward_speed:.2f}m/s v_follow_max={cfg.follow_max_speed:.2f}m/s")

    def on_local_sensors(self, msg):
        self.local_snapshot = parse_snapshot(msg.data)
        self.last_snapshot_at = time.monotonic()

    def on_contact(self, msg):
        active = bool(msg.data)
        if active and not self.contact_active:
            now = time.monotonic()
            self.collision_backoff_until = now + COLLISION_BACKOFF_S
            self.collision_restart_at = (self.collision_backoff_until +
                                         random.uniform(0.0, 1.0))
        self.contact_active = active

    def on_tick(self):
        now = time.monotonic()
        if (self.contact_active and self.collision_restart_at > 0.0 and
                now >= self.collision_restart_at):
            # Do not resume normal following into a still-active contact.
            # Repeat the recovery cycle until the bumper clears.
            self.collision_backoff_until = now + COLLISION_BACKOFF_S
            self.collision_restart_at = (self.collision_backoff_until +
                                         random.uniform(0.0, 1.0))
        if self.last_snapshot_at is None:
            health = "NO_SNAPSHOT"
        elif now - self.last_snapshot_at > FRESH_S:
            health = "STALE_SNAPSHOT"
        elif self.local_snapshot is None:
            health = "INVALID_SNAPSHOT"
        else:
            health = "OK"
        self.sensor_health_pub.publish(String(data=health))
        recovery = collision_recovery_command(
            now, self.collision_backoff_until, self.collision_restart_at)
        if recovery is not None:
            v, w, mode = recovery
        elif health != "OK":
            v, w, mode = 0.0, 0.0, "SENSOR_FAILSAFE"
        else:
            rear_d, rear_b, front_d, front_b, ranges = self.local_snapshot
            v, w, mode = safe_command(rear_d, rear_b, front_d, front_b,
                                      ranges, self.cfg)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        self.mode_pub.publish(String(data=mode))
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(f"{self.name_} {mode} v={v:.2f} w={w:+.2f}")

    def stop(self):
        # Ctrl+C can invalidate the ROS context before finally runs.
        # The Webots bridge also has a 0.7s dead-command watchdog, so it is
        # safer to skip this final publish than raise RCLError on shutdown.
        try:
            if rclpy.ok(context=self.context):
                self.cmd_pub.publish(Twist())
        except Exception:
            # Context already closed during Ctrl+C; shutdown is underway.
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", nargs="+", choices=[f"R{i}" for i in range(1,6)],
                        default=[f"R{i}" for i in range(1,6)])
    args = parser.parse_args()
    cfg = load_config()
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for name in dict.fromkeys(args.robots):
            node = SafeFollower(name, cfg[name])
            nodes.append(node)
            executor.add_node(node)
        print("Safe followers running: " + ", ".join(args.robots), flush=True)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.stop()
        # Do not spin a shutdown context; earlier versions failed here or
        # when trying to publish a final zero Twist after Ctrl+C.
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
