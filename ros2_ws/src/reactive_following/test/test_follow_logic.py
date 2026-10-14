"""ROS- and Webots-independent sanity checks for the reactive law."""
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "reactive_following"))
from follow_logic import ControlParameters, command_for_detection


class TestFollowLogic(unittest.TestCase):
    def setUp(self):
        self.cfg = ControlParameters()

    def test_no_target_uses_base_curvature(self):
        self.assertEqual(command_for_detection(False, None, self.cfg),
                         (0.16, -0.20, "BASE_CURVE"))

    def test_target_ahead_drives_straight(self):
        self.assertEqual(command_for_detection(True, 0.0, self.cfg),
                         (0.16, 0.0, "FOLLOW"))

    def test_left_bearing_turns_left(self):
        v, w, state = command_for_detection(True, 0.25, self.cfg)
        self.assertEqual(state, "FOLLOW")
        self.assertAlmostEqual(w, 0.45)

    def test_right_bearing_turns_right(self):
        v, w, state = command_for_detection(True, -0.25, self.cfg)
        self.assertAlmostEqual(w, -0.45)

    def test_angular_speed_is_clamped(self):
        self.assertEqual(command_for_detection(True, 0.9, self.cfg)[1], 0.90)
        self.assertEqual(command_for_detection(True, -0.9, self.cfg)[1], -0.90)

    def test_nonfinite_target_not_chased(self):
        self.assertEqual(command_for_detection(True, math.nan, self.cfg)[2], "BASE_CURVE")

    def test_speed_without_distance_is_constant(self):
        self.assertEqual(command_for_detection(True, 0.6, self.cfg)[0],
                         command_for_detection(False, None, self.cfg)[0])

    def test_distance_at_two_times_robot_size(self):
        # 2x robot size (2 * 0.25m = 0.50m) maintains nominal forward speed
        v, _, mode = command_for_detection(True, 0.0, self.cfg, distance_m=0.50)
        self.assertEqual(mode, "FOLLOW")
        self.assertAlmostEqual(v, 0.16)

    def test_distance_too_close_stops(self):
        # Distance closer than 0.30 m causes the robot to stop to prevent collision
        v, _, mode = command_for_detection(True, 0.0, self.cfg, distance_m=0.25)
        self.assertEqual(mode, "FOLLOW")
        self.assertEqual(v, 0.0)

    def test_distance_far_speeds_up(self):
        # Distance farther than 0.50 m increases speed to catch up, clamped at max
        v, _, mode = command_for_detection(True, 0.0, self.cfg, distance_m=1.00)
        self.assertEqual(mode, "FOLLOW")
        self.assertEqual(v, self.cfg.max_forward_speed)


if __name__ == "__main__":
    unittest.main()
