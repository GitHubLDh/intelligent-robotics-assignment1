"""Regression: corner-to-diagonal transition must not reverse steering.

This is an isolated logic test, not Webots collision validation.
"""
import unittest
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_controller'))
from safe_logic import SafeParameters, safe_command


def sector_scan(f,fl,fr,l=.60,r=.55,b=2.5):
    scan=[2.5]*360
    # Local scan index 180 is forward; 220=front-left, 140=front-right.
    def patch(mid,dist,half=7):
        for i in range(mid-half,mid+half+1):
            scan[i%360]=dist
    patch(180,f,12)
    patch(220,fl,9)
    patch(140,fr,9)
    patch(270,l,10)
    patch(90,r,10)
    patch(0,b,10)
    return scan

class AvoidanceChatterTests(unittest.TestCase):
    cfg=SafeParameters(forward_speed=.14,base_angular_speed=.24)

    def test_corner_diagonal_boundary_keeps_right_turn(self):
        # First scan is genuinely near a corner; second still has a close
        # diagonal. The tuned controller must not reverse the turn sign.
        a=safe_command(None,None,None,None,sector_scan(.64,.44,.30),self.cfg)
        b=safe_command(None,None,None,None,sector_scan(1.06,.44,.30),self.cfg)
        self.assertEqual(a[2],'CORNER_CLEAR_TURN')
        self.assertEqual(b[2],'AVOID_DIAGONAL')
        self.assertEqual((a[0],b[0]),(0.0,0.0))
        self.assertLess(a[1],0)
        self.assertLess(b[1],0)

    def test_critical_right_obstacle_backoff_not_opposite_turn(self):
        # Turning left only for a single close right return caused yaw reversals.
        # Now, too-close side + free rear implies straight, cautious backoff.
        v,w,mode=safe_command(None,None,None,None,
             sector_scan(1.06,.44,.30,l=1.0,r=.17),self.cfg)
        self.assertLess(v,0.0)
        self.assertEqual(w,0.0)
        self.assertEqual(mode,'ESCAPE_SIDE')

    def test_normal_headon_still_right(self):
        v,w,mode=safe_command(None,None,.60,0,
             sector_scan(1.5,1.8,1.8,l=1.8,r=1.8),self.cfg)
        self.assertEqual(mode,'YIELD')
        self.assertLess(w,0.0)

    def test_open_scene_base_curve_unchanged(self):
        v,w,mode=safe_command(None,None,None,None,[2.5]*360,self.cfg)
        self.assertEqual(mode,'BASE_CURVE')
        self.assertAlmostEqual(v,.14)
        self.assertAlmostEqual(w,.24)

if __name__ == '__main__':
    unittest.main()
