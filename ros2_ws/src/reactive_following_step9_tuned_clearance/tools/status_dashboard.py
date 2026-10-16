#!/usr/bin/env python3
"""Live terminal dashboard: current mode/v/w for all 5 robots at once.

Run this in its own terminal window, alongside run_trains.sh's Webots
window - much easier to watch this table update in real time next to the
simulation than to cross-reference the scrolling, interleaved log file
while robots are moving.

Usage (after sourcing ROS2):
    python3 -u tools/status_dashboard.py
"""
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

ROBOTS = [f"R{i}" for i in range(1, 6)]


class Dashboard(Node):
    def __init__(self):
        super().__init__('status_dashboard')
        self.state = {name: {'mode': '-', 'v': 0.0, 'w': 0.0} for name in ROBOTS}
        for name in ROBOTS:
            ns = f"/{name}"
            self.create_subscription(String, ns + "/follow_mode",
                                     self._mode_callback(name), 10)
            self.create_subscription(Twist, ns + "/cmd_vel",
                                     self._cmd_callback(name), 10)
        self.create_timer(0.1, self.render)

    def _mode_callback(self, name):
        def cb(msg):
            self.state[name]['mode'] = msg.data
        return cb

    def _cmd_callback(self, name):
        def cb(msg):
            self.state[name]['v'] = msg.linear.x
            self.state[name]['w'] = msg.angular.z
        return cb

    def render(self):
        out = ["\x1b[2J\x1b[H"]  # clear screen, cursor to top-left
        out.append(f"{'Robot':<6}{'Mode':<22}{'v (m/s)':>9}{'w (rad/s)':>11}")
        out.append("-" * 48)
        for name in ROBOTS:
            s = self.state[name]
            out.append(f"{name:<6}{s['mode']:<22}{s['v']:>9.2f}{s['w']:>11.2f}")
        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
