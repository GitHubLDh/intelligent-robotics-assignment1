#!/usr/bin/env python3
"""Live terminal dashboard: current mode/v/w for all 5 robots at once.

Run this in its own terminal window, alongside run_trains.sh's Webots
window - much easier to watch this table update in real time next to the
simulation than to cross-reference the scrolling, interleaved log file
while robots are moving.

Pausing Webots stops the sensor snapshots, so the controllers correctly fall
to SENSOR_FAILSAFE and command zero. That is useless to look at - the moment
you pause to study what the robots were doing, the table blanks. So a
SENSOR_FAILSAFE does not overwrite the display: the last real mode, v and w
stay on screen and the row is marked HELD.

Usage (after sourcing ROS2):
    python3 -u tools/status_dashboard.py
"""
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

ROBOTS = [f"R{i}" for i in range(1, 6)]
# Published whenever the snapshot is missing or stale - which is exactly what
# pausing the simulation looks like from the controller's side.
STALE_MODE = 'SENSOR_FAILSAFE'


class Dashboard(Node):
    def __init__(self):
        super().__init__('status_dashboard')
        # 'pending_*' always tracks the latest cmd_vel; it is only promoted to
        # the displayed v/w when a real mode arrives with it. cmd_vel is
        # published just before follow_mode, so without this the zero command
        # of the first stale tick would land before we knew to hold.
        self.state = {name: {'mode': '-', 'v': 0.0, 'w': 0.0,
                             'pending_v': 0.0, 'pending_w': 0.0,
                             'held': False} for name in ROBOTS}
        for name in ROBOTS:
            ns = f"/{name}"
            self.create_subscription(String, ns + "/follow_mode",
                                     self._mode_callback(name), 10)
            self.create_subscription(Twist, ns + "/cmd_vel",
                                     self._cmd_callback(name), 10)
        self.create_timer(0.1, self.render)

    def _mode_callback(self, name):
        def cb(msg):
            state = self.state[name]
            if msg.data == STALE_MODE and state['mode'] not in ('-', STALE_MODE):
                # Paused or dropped out: freeze the row rather than blanking it.
                state['held'] = True
                return
            state['mode'] = msg.data
            state['held'] = False
            state['v'] = state['pending_v']
            state['w'] = state['pending_w']
        return cb

    def _cmd_callback(self, name):
        def cb(msg):
            self.state[name]['pending_v'] = msg.linear.x
            self.state[name]['pending_w'] = msg.angular.z
        return cb

    def render(self):
        out = ["\x1b[2J\x1b[H"]  # clear screen, cursor to top-left
        out.append(f"{'Robot':<6}{'Mode':<22}{'v (m/s)':>9}{'w (rad/s)':>11}  ")
        out.append("-" * 56)
        for name in ROBOTS:
            s = self.state[name]
            flag = '  HELD' if s['held'] else ''
            out.append(f"{name:<6}{s['mode']:<22}{s['v']:>9.2f}{s['w']:>11.2f}{flag}")
        if any(s['held'] for s in self.state.values()):
            out.append("")
            out.append("HELD = no fresh sensor data (simulation paused);"
                       " showing the last live values.")
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
