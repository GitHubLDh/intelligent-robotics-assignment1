"""Five independent ROS 2/UDP bridge nodes in one Windows-friendly process.

For Ri: /Ri/cmd_vel (Twist) -> UDP 1500i -> Webots wheel controller.
        camera recognition -> /Ri/target_visible, /Ri/target, /Ri/visible_count.

Each robot uses a distinct Webots UDP socket and ROS 2 node. No shared robot
poses, central decision maker or data transfer between robot controllers.
"""

import argparse
import json
import socket
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class RobotBridge(Node):
    def __init__(self, robot_name):
        super().__init__(f"{robot_name.lower()}_webots_bridge")
        self.robot_name = robot_name
        self.destination = ("127.0.0.1", 15000 + int(robot_name[1:]))
        self.ack = f"{robot_name}:ok".encode("ascii")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)

        self.linear = self.angular = 0.0
        self.last_cmd_time = 0.0
        self.last_ack_time = None
        self.last_sensor_time = None
        self.started_at = time.monotonic()
        self.last_warning_time = 0.0
        self.connected = False
        self.last_visible = None

        ns = f"/{robot_name}"
        self.create_subscription(Twist, f"{ns}/cmd_vel", self.on_command, 10)
        self.visible_pub = self.create_publisher(Bool, f"{ns}/target_visible", 10)
        self.target_pub = self.create_publisher(Vector3Stamped, f"{ns}/target", 10)
        self.count_pub = self.create_publisher(Int32, f"{ns}/visible_count", 10)
        self.create_timer(0.1, self.on_tick)
        self.get_logger().info(f"{robot_name}: awaiting Webots on UDP {self.destination[1]}")

    def on_command(self, msg):
        self.linear = float(msg.linear.x)
        self.angular = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()

    def publish_sensor(self, data):
        visible = bool(data.get("visible", False))
        self.visible_pub.publish(Bool(data=visible))
        self.count_pub.publish(Int32(data=max(0, int(data.get("count", 0)))))
        if visible:
            target = Vector3Stamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = f"{self.robot_name}/front_camera"
            target.vector.x = float(data["distance_m"])
            target.vector.y = float(data["bearing_rad"])
            self.target_pub.publish(target)
        if visible != self.last_visible:
            self.last_visible = visible
            self.get_logger().info("Target acquired" if visible else "Target not visible")

    def on_tick(self):
        now = time.monotonic()
        if now - self.last_cmd_time > 0.7:
            v = w = 0.0
        else:
            v, w = self.linear, self.angular
        try:
            self.sock.sendto(json.dumps({"linear": v, "angular": w}).encode("utf-8"), self.destination)
        except OSError as exc:
            self.get_logger().error(f"UDP send failed: {exc}")

        while True:
            try:
                packet, sender = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            if sender != self.destination:
                continue
            if packet == self.ack:
                self.last_ack_time = now
                if not self.connected:
                    self.connected = True
                    self.get_logger().info(f"Connected to Webots {self.robot_name}")
                continue
            try:
                data = json.loads(packet.decode("utf-8"))
                if data.get("type") != "target":
                    continue
                self.last_sensor_time = now
                self.publish_sensor(data)
            except (ValueError, TypeError, KeyError, UnicodeError) as exc:
                self.get_logger().warning(f"Invalid sensor packet: {exc}")

        # Fail-safe: a stopped camera must not leave stale positive visibility.
        if self.last_sensor_time is None or now - self.last_sensor_time > 0.6:
            self.publish_sensor({"visible": False, "count": 0})

        missing = self.last_ack_time is None or now - self.last_ack_time > 1.5
        if missing and now - self.started_at > 3.0 and now - self.last_warning_time > 3.0:
            self.connected = False
            self.last_warning_time = now
            self.get_logger().warning("No reply from Webots: check world and controller")

    def close(self):
        try:
            self.sock.sendto(b'{"linear":0,"angular":0}', self.destination)
        except OSError:
            pass
        self.sock.close()


def main(args=None):
    import sys
    from rclpy.utilities import remove_ros_args

    clean_args = remove_ros_args(args=sys.argv[1:] if args is None else args)
    parser = argparse.ArgumentParser(description="Start ROS 2 bridges for five Webots robots")
    parser.add_argument("--robots", nargs="+", choices=[f"R{i}" for i in range(1, 6)],
                        default=[f"R{i}" for i in range(1, 6)],
                        help="Subset to start, e.g. --robots R1 R2 (default: all)")
    args_parsed, _ = parser.parse_known_args(clean_args)

    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for name in dict.fromkeys(args_parsed.robots):
            node = RobotBridge(name)
            nodes.append(node)
            executor.add_node(node)
        print("ROS 2 bridges running: " + ", ".join(n.robot_name for n in nodes), flush=True)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.close()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
