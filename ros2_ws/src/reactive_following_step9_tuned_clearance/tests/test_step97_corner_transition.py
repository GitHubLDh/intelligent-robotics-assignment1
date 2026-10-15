"""Regression for mode-to-mode steering flips shown in R2's Step 9.6 logs.

Only local one-frame LiDAR readings are passed to the controller.
These are logic tests, not contact-physics simulations.
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_controller'))
from safe_logic import SafeParameters, safe_command


def sample(front=2.5, front_left=2.5, front_right=2.5,
           left=2.5, right=2.5, rear=2.5):
    out = [2.5] * 360
    def patch(index, d, extent):
        for i in range(index - extent, index + extent + 1):
            out[i % 360] = d
    patch(180, front, 10)
    patch(220, front_left, 8)
    patch(140, front_right, 8)
    patch(270, left, 8)
    patch(90, right, 8)
    patch(0, rear, 10)
    return out


class Step97Regression(unittest.TestCase):
    cfg = SafeParameters(forward_speed=.14, base_angular_speed=.24)

    def cmd(self, **kwargs):
        return safe_command(None, None, None, None, sample(**kwargs), self.cfg)

    def test_brake_front_diagonal_no_sign_flip(self):
        # Prior code: BRAKE_TURN = -0.8 but AVOID_DIAGONAL = +0.8.
        kwargs = dict(front_right=.29, front_left=.90, left=.8, right=.7)
        cmds = [self.cmd(front=front, **kwargs) for front in (.26, .46, .90, 1.40)]
        modes = [m for _, _, m in cmds]
        self.assertEqual(modes[0], 'BRAKE_TURN')
        self.assertEqual(modes[-1], 'AVOID_DIAGONAL')
        self.assertTrue(all(m in ('BRAKE_TURN', 'CORNER_CLEAR_TURN', 'AVOID_FRONT',
                                   'AVOID_DIAGONAL') for m in modes), modes)
        self.assertTrue(all(v == 0.0 and w < 0 for v, w, _ in cmds), cmds)

    def test_front_and_side_modes_keep_one_turn_direction(self):
        # The previous left/right switching is deliberately removed. A near
        # right obstacle at .25m still permits a circular center pivot.
        kwargs = dict(left=.95, right=.25)
        cmds = [self.cmd(front=front, **kwargs) for front in (.26, .49, 1.4)]
        self.assertEqual([m for _,_,m in cmds],
                         ['BRAKE_TURN', 'AVOID_FRONT', 'AVOID_SIDE'])
        self.assertTrue(all(v == 0.0 and w < 0 for v,w,_ in cmds), cmds)

    def test_small_side_perturbations_keep_right_preference(self):
        # Near-symmetric side ranges should not change the turn direction.
        cmds = [self.cmd(front=.46, left=l, right=r)
                for l,r in ((.36,.34),(.34,.36),(.42,.32),(.32,.42))]
        self.assertTrue(all(w < 0 for _,w,_ in cmds), cmds)

    def test_red_headon_keeps_right_passing_direction(self):
        v,w,mode=safe_command(None,None,.63,0,
            sample(left=.95,right=.25),self.cfg)
        self.assertTrue(mode.startswith('YIELD'))
        self.assertLess(w,0.0)

    def test_open_space_is_still_base_curve(self):
        v,w,mode=self.cmd()
        self.assertEqual((v,w,mode),(.14,.24,'BASE_CURVE'))

if __name__ == '__main__':
    unittest.main()
