"""Static multi-robot checks + local sensing tests (ROS/Webots not required)."""

import json
import math
from pathlib import Path
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "udp_diff_driver"))
from perception import select_nearest_robot  # noqa: E402


class MockRecognition:
    def __init__(self, model, x, y, z=0.0):
        self.model, self.position = model, (x, y, z)

    def getModel(self):
        return self.model

    def getPosition(self):
        return self.position


class TestLocalPerception(unittest.TestCase):
    def test_nearest_visible_robot_wins(self):
        objects = [MockRecognition("simple_diff_robot", 2.2, 0.1),
                   MockRecognition("simple_diff_robot", 0.8, 0.0),
                   MockRecognition("wall", 0.3, 0.0)]
        target = select_nearest_robot(objects)
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[0], 0.8)
        self.assertEqual(target[2], 2)

    def test_excludes_robot_outside_camera_cone(self):
        # 60 degrees > 50-degree half FOV
        obj = MockRecognition("simple_diff_robot", 1.0, math.tan(math.radians(60)))
        self.assertIsNone(select_nearest_robot([obj]))

    def test_excludes_robot_beyond_four_metres(self):
        self.assertIsNone(select_nearest_robot([MockRecognition("simple_diff_robot", 4.1, 0.0)]))

    def test_positive_left_bearing(self):
        obj = MockRecognition("simple_diff_robot", 1, 1)
        self.assertAlmostEqual(select_nearest_robot([obj])[1], math.pi / 4)


class TestSceneConfig(unittest.TestCase):
    def test_five_robots_use_controller(self):
        world = (ROOT / "worlds" / "reactive_following.wbt").read_text(encoding="utf-8")
        robot_blocks = re.findall(r"SimpleDiffRobot\s*\{(.*?)\n\}", world, re.S)
        self.assertEqual(len(robot_blocks), 5)
        for i, block in enumerate(robot_blocks, 1):
            self.assertIn(f'name "R{i}"', block)
            self.assertIn('controller "udp_diff_driver"', block)

    def test_individual_speed_and_turning(self):
        cfg = json.loads((ROOT / "config" / "robots.json").read_text(encoding="utf-8"))
        self.assertEqual(set(cfg), {f"R{i}" for i in range(1, 6)})
        self.assertEqual(len({x["forward_speed"] for x in cfg.values()}), 5)
        self.assertEqual(len({x["base_angular_speed"] for x in cfg.values()}), 5)
        for item in cfg.values():
            self.assertGreater(item["forward_speed"], 0)
            self.assertLess(item["forward_speed"], 0.25)
            self.assertLess(abs(item["base_angular_speed"]), item["max_angular_speed"])


if __name__ == "__main__":
    unittest.main()
