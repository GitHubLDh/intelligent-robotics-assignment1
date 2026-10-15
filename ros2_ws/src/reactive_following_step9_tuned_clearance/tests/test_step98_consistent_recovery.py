"""Step 9.8: regressions from R2's logged -0.8/+0.8 turning loop.

Only instantaneous LiDAR frames (no history/target state) are used.
These tests are NOT Webots collision-physics validation.
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_controller'))
from safe_logic import SafeParameters, safe_command
from test_step97_corner_transition import sample


class StatelessRecoveryTests(unittest.TestCase):
    cfg = SafeParameters(forward_speed=.14, base_angular_speed=.24)

    def decide(self, **kwargs):
        return safe_command(None, None, None, None, sample(**kwargs), self.cfg)

    def test_brake_corner_diagonal_forward_keep_clockwise(self):
        frames = [
            dict(front=.25, front_left=.80, front_right=.30, left=.80, right=.25),
            dict(front=.52, front_left=.85, front_right=.33, left=.90, right=.30),
            dict(front=1.05, front_left=.85, front_right=.28, left=1.2, right=.37),
            dict(front=.49, front_left=.64, front_right=.48, left=.78, right=.43),
            dict(front=.84, front_left=.62, front_right=.61, left=.81, right=.65),
        ]
        results = [self.decide(**frame) for frame in frames]
        self.assertIn('BRAKE_TURN', [mode for _, _, mode in results])
        self.assertIn('AVOID_DIAGONAL', [mode for _, _, mode in results])
        self.assertTrue(all(v >= 0 and w < 0 for v, w, _ in results), results)
        self.assertTrue(all(v == 0 or (v <= .075 and m == 'CLEAR_FORWARD_ARC')
                            for v, w, m in results), results)

    def test_clear_corridor_advances_cautiously_instead_of_spin(self):
        v, w, mode = self.decide(front=.79, front_left=.71,
                                  front_right=.73, left=.90, right=.75)
        self.assertEqual(mode, 'CLEAR_FORWARD_ARC')
        self.assertGreater(v, 0)
        self.assertLess(v, self.cfg.forward_speed)
        self.assertLess(w, 0)

    def test_smooth_return_to_base_curvature(self):
        cmds = [self.decide(front=f, front_left=f, front_right=f)
                for f in (.92, 1.01, 1.10)]
        self.assertEqual(cmds[0][2], 'BASE_CLEAR_BLEND')
        self.assertEqual(cmds[2][2], 'BASE_CURVE')
        self.assertLess(cmds[0][1], cmds[1][1])
        self.assertLess(cmds[1][1], cmds[2][1])
        self.assertLess(max(abs(cmds[i+1][1] - cmds[i][1]) for i in range(2)), .36)

    def test_critical_right_side_does_not_flip_yaw(self):
        v, w, mode = self.decide(front=1.06, front_left=.44, front_right=.30,
                                  left=1., right=.17, rear=1.3)
        self.assertEqual(mode, 'ESCAPE_SIDE')
        self.assertLess(v, 0)
        self.assertEqual(w, 0)

    def test_critical_side_with_blocked_rear_stops(self):
        v, w, mode = self.decide(front=1.06, front_left=.44, front_right=.30,
                                  left=1., right=.17, rear=.20)
        self.assertEqual((v, w, mode), (0, 0, 'SIDE_BLOCKED'))

    def test_safety_on_bad_lidar_snapshot(self):
        v, w, mode = safe_command(None, None, None, None,
                                   [float('nan')] * 360, self.cfg)
        self.assertEqual((v, w, mode), (0., 0., 'SENSOR_FAILSAFE'))

    def test_rear_mark_hold_not_changed(self):
        v, w, mode = safe_command(.56, .03, None, None,
                                  sample(front=2.5), self.cfg)
        self.assertEqual((v, w, mode), (0., 0., 'FOLLOW_HOLD'))


if __name__ == '__main__':
    unittest.main()
