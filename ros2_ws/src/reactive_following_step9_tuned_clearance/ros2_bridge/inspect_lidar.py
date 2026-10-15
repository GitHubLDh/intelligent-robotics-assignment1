"""Print instantaneous local LiDAR clearances (diagnostic only).

Run from the project root: python ros2_bridge/inspect_lidar.py --robot R1
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_controller'))
from safe_logic import lidar_sectors

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node


class Viewer(Node):
    def __init__(self, name):
        super().__init__(f'{name.lower()}_lidar_inspector')
        self.total = 0
        self.create_subscription(LaserScan, f'/{name}/scan', self.on_scan, 10)
        print(f'Inspecting /{name}/scan; no robot commands are sent.', flush=True)

    def on_scan(self, msg):
        self.total += 1
        if self.total % 8:
            return
        s = lidar_sectors(msg.ranges)
        if s is None:
            print('INVALID LIDAR DATA',flush=True)
            return
        print('  '.join(f'{label}: {s[key]:.2f}m' for label,key in [
            ('FRONT','front'),('FL','front_left'),('FR','front_right'),
            ('LEFT','left'),('RIGHT','right'),('REAR','rear')]),flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--robot', choices=[f'R{i}' for i in range(1,6)],default='R1')
    args=parser.parse_args()
    rclpy.init()
    node=Viewer(args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    main()
