"""Step 9.9 specific tuning regressions: isolated controller decisions."""
import sys
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_controller'))
from safe_logic import SafeParameters, safe_command
from test_step97_corner_transition import sample


class ClearanceTuneTests(unittest.TestCase):
    cfg = SafeParameters(forward_speed=.14, base_angular_speed=.24)

    def decide(self, **kwargs):
        return safe_command(None, None, None, None, sample(**kwargs), self.cfg)

    def test_wall_one_metre_away_does_not_stop(self):
        v, w, m = self.decide(front=1.12, front_left=1.05,
                              front_right=1.07, left=.90, right=.93)
        self.assertIn(m, ('BASE_CURVE','BASE_CLEAR_BLEND'))
        self.assertAlmostEqual(v, .14)
        self.assertGreater(w, 0)  # already blending toward R2's normal curve

    def test_diagonals_near_metre_do_not_force_pivot(self):
        v, w, m = self.decide(front=.96, front_left=.87,
                              front_right=.91, left=.86, right=.80)
        self.assertGreater(v, 0)
        self.assertNotIn(m, ('CORNER_CLEAR_TURN','CLEAR_FORWARD_TURN','AVOID_DIAGONAL'))

    def test_safe_corridor_turns_while_moving(self):
        v, w, m = self.decide(front=.77, front_left=.68,
                              front_right=.71, left=.82, right=.83)
        self.assertEqual(m,'CLEAR_FORWARD_ARC')
        self.assertGreater(v,.035)  # Faster than step 9.8
        self.assertLessEqual(v,.075)
        self.assertAlmostEqual(w,-.85)

    def test_near_wall_pivots_more_quickly(self):
        v, w, m = self.decide(front=.49, front_left=.65,
                              front_right=.65, left=.78, right=.80)
        self.assertEqual(m,'AVOID_FRONT')
        self.assertEqual(v,0)
        self.assertAlmostEqual(w,-1.05)

    def test_emergency_stop_remains_safety_first(self):
        v,w,m=self.decide(front=.14, front_left=.27,front_right=.28,
                           left=.23,right=.23,rear=.20)
        self.assertEqual((v,w,m),(0,0,'EMERGENCY_STOP'))

    def test_follow_hold_remains_unchanged(self):
        v,w,m=safe_command(.55,.02,None,None,sample(),self.cfg)
        self.assertEqual((v,w,m),(0,0,'FOLLOW_HOLD'))

    def test_config_five_robots_are_consistent(self):
        import json
        conf=json.loads((ROOT/'config/robots.json').read_text())
        for robot, cfg in conf.items():
            self.assertEqual(cfg['lidar_front_avoid'],.52,robot)
            self.assertEqual(cfg['lidar_slow_range'],.82,robot)
            self.assertEqual(cfg['avoid_angular_speed'],1.05,robot)

if __name__ == '__main__': unittest.main()
