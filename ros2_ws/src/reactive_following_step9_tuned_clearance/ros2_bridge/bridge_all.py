"""Five independent ROS 2/UDP bridge nodes in one Windows-friendly process.

For Ri: /Ri/cmd_vel (Twist) -> UDP 1500i -> Webots wheel controller.
        camera -> /Ri/target_visible, /Ri/target; LiDAR -> /Ri/scan.
        /Ri/local_sensors carries both in one synchronized snapshot.

Each robot uses a distinct Webots UDP socket and ROS 2 node. No shared robot
poses, central decision maker or data transfer between robot controllers.
"""

import argparse
import json
import socket
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import LaserScan
import math
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray, String
from sensor_packet import sensor_packet


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
        self.snapshot_pub = self.create_publisher(Float64MultiArray, f"{ns}/local_sensors", 10)
        self.visible_pub = self.create_publisher(Bool, f"{ns}/target_visible", 10)
        # target_visible means a green rear marker; robot_visible means
        # either front RED or rear GREEN (not a specific robot identity).
        self.robot_visible_pub = self.create_publisher(Bool, f"{ns}/robot_visible", 10)
        self.camera_raw_pub = self.create_publisher(Int32, f"{ns}/camera_raw_count", 10)
        self.camera_models_pub = self.create_publisher(String, f"{ns}/camera_models", 10)
        self.camera_classification_pub = self.create_publisher(String, f"{ns}/camera_classification", 10)
        self.target_pub = self.create_publisher(Vector3Stamped, f"{ns}/target", 10)
        self.count_pub = self.create_publisher(Int32, f"{ns}/visible_count", 10)
        self.front_pub = self.create_publisher(Vector3Stamped, f"{ns}/front_marker", 10)
        self.scan_pub = self.create_publisher(LaserScan, f"{ns}/scan", 10)
        self.contact_pub = self.create_publisher(Bool, f"{ns}/contact", 10)
        self.contact_events_pub = self.create_publisher(Int32, f"{ns}/contact_events", 10)
        self.sim_time_pub = self.create_publisher(Float64, f"{ns}/sim_time", 10)
        # Communication-only diagnostics, never used for navigation.
        self.bridge_health_pub = self.create_publisher(String, f"{ns}/bridge_health", 10)
        self.create_timer(0.05, self.on_tick)
        self.get_logger().info(f"{robot_name}: awaiting Webots on UDP {self.destination[1]}")

    def on_command(self, msg):
        self.linear = float(msg.linear.x)
        self.angular = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()

    def publish_sensor(self, data):
        # One atomic snapshot prevents mixing a camera frame with
        # previous-cycle LiDAR/marker readings.
        self.snapshot_pub.publish(Float64MultiArray(data=sensor_packet(data)))
        visible = bool(data.get("visible", False))
        self.visible_pub.publish(Bool(data=visible))
        self.robot_visible_pub.publish(Bool(data=visible or bool(data.get("front_visible", False))))
        self.camera_raw_pub.publish(Int32(data=max(0, int(data.get("raw_recognition_count", 0)))))
        models = data.get("recognition_models", [])
        if not isinstance(models, list):
            models = []
        self.camera_models_pub.publish(String(data=", ".join(str(x) for x in models)))
        self.camera_classification_pub.publish(String(data=str(data.get("camera_classification", "no telemetry"))))
        self.count_pub.publish(Int32(data=max(0, int(data.get("count", 0)))))
        if visible:
            target = Vector3Stamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = f"{self.robot_name}/front_camera"
            target.vector.x = float(data["distance_m"])
            target.vector.y = float(data["bearing_rad"])
            self.target_pub.publish(target)
        # Sensor packet contains a synchronized snapshot from one robot.
        # For optional measurements, NaN means no marker; controllers validate.
        marker = Vector3Stamped()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = f"{self.robot_name}/front_camera"
        import math
        marker.vector.x = float(data["front_distance_m"]) if data.get("front_visible") else math.nan
        marker.vector.y = float(data["front_bearing_rad"]) if data.get("front_visible") else math.nan
        self.front_pub.publish(marker)
        # Standard ROS 2 LaserScan: 360 local depth rays. Index 180 = +X.
        values = data.get('lidar_ranges_m')
        if isinstance(values, list) and len(values) == 360:
            scan = LaserScan()
            scan.header.stamp = marker.header.stamp
            scan.header.frame_id = f"{self.robot_name}/safety_lidar"
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / 360.0
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.time_increment = 0.0
            scan.scan_time = 0.064
            scan.range_min = 0.05
            scan.range_max = 2.5
            scan.ranges = [float(x) for x in values]
            self.scan_pub.publish(scan)
        if "sim_time_s" in data:
            self.contact_pub.publish(Bool(data=bool(data.get("contact", False))))
            self.contact_events_pub.publish(Int32(data=int(data.get("contact_episodes", 0))))
            # Publish simulation time after associated telemetry in this tick.
            self.sim_time_pub.publish(Float64(data=float(data["sim_time_s"])))
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
                packet, sender = self.sock.recvfrom(65535)
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

        # Distinguish a stopped/paused Webots simulation from a controller
        # classification failure. Sensor health remains a separate topic.
        if self.last_sensor_time is None:
            state = "NO_SENSOR_PACKET"
        elif now - self.last_sensor_time <= 0.6:
            state = "OK"
        elif self.last_ack_time is not None and now - self.last_ack_time <= 1.5:
            state = "ACK_ONLY_NO_SENSORS"
        else:
            state = "WEBOTS_TELEMETRY_STALE"
        self.bridge_health_pub.publish(String(data=state))
        # Fail-safe: a stopped camera must not leave stale positive visibility.
        if self.last_sensor_time is None or now - self.last_sensor_time > 0.6:
            # Publish fail-safe obstacle readings (0m) on stale bridge data.
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


def main():
    parser = argparse.ArgumentParser(description="Start ROS 2 bridges for five Webots robots")
    parser.add_argument("--robots", nargs="+", choices=[f"R{i}" for i in range(1, 6)],
                        default=[f"R{i}" for i in range(1, 6)],
                        help="Subset to start, e.g. --robots R1 R2 (default: all)")
    args = parser.parse_args()

    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for name in dict.fromkeys(args.robots):
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
