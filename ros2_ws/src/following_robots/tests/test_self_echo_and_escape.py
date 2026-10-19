
"""Conservative geometry and stateless corner-control regression tests.

These do not simulate Webots LiDAR rendering or contact physics.
"""
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_controller'))
from safe_logic import safe_command,SafeParameters

def scan(default=2.5, *, f=2.5, fl=2.5, fr=2.5,
         l=2.5, r=2.5, rear=2.5, bl=2.5, br=2.5):
    result=[default]*360
    for i in range(360):
        a=abs((i*math.pi/180)-math.pi)
        if a<=math.radians(19):result[i]=f
        if math.radians(31)<=i*math.pi/180-math.pi<=math.radians(49):result[i]=fl
        if -math.radians(49)<=i*math.pi/180-math.pi<=-math.radians(31):result[i]=fr
        if 75<=i-180<=105:result[i]=l
        if -105<=i-180<=-75:result[i]=r
        if 160<=abs(i-180)<=180:result[i]=rear
        if 126<=i-180<=154:result[i]=bl
        if -154<=i-180<=-126:result[i]=br
    return result

class GeometryTests(unittest.TestCase):
    def test_lidar_above_own_rear_marker_at_near_distance(self):
        world_z=0.085+0.070
        lowest_rear_ray=world_z-0.12*math.tan(0.04)
        rear_marker_top=0.085+0.005+0.080/2
        self.assertGreater(lowest_rear_ray,rear_marker_top+0.01)

    def test_low_wall_in_scanning_height_nearby(self):
        world_z=0.085+0.070
        for distance in (0.25,0.40,0.75):
            for pitch in (-.04,0,.04):
                hit=world_z+distance*math.tan(pitch)
                self.assertGreater(hit,0)
                self.assertLess(hit,0.19)

class CornerTests(unittest.TestCase):
    cfg=SafeParameters(forward_speed=.14,base_angular_speed=.24)
    def test_sign_consistent_through_small_right_left_changes(self):
        for side_l,side_r in ((.38,.35),(.35,.38),(.30,.41),(.41,.30)):
            v,w,m=safe_command(None,None,None,None,scan(f=.4,fl=.31,fr=.32,l=side_l,r=side_r),self.cfg)
            self.assertEqual(m,'CORNER_CLEAR_TURN',m)
            self.assertEqual(v,0)
            self.assertLess(w,0)
    def test_can_backoff_with_rear_clear(self):
        v,w,m=safe_command(None,None,None,None,scan(f=.29,fl=.20,fr=.2,l=.18,r=.22,rear=1.5,bl=1.,br=1.),self.cfg)
        self.assertEqual(m,'CORNER_BACKOFF',m)
        self.assertLess(v,0)
        self.assertEqual(w,0)
    def test_rear_obstruction_prevents_backoff(self):
        v,w,m=safe_command(None,None,None,None,scan(f=.29,fl=.20,fr=.2,l=.18,r=.22,rear=.12,bl=1.,br=1.),self.cfg)
        self.assertEqual(m,'CORNER_BLOCKED')
        self.assertEqual((v,w),(0,0))
    def test_headon_still_right(self):
        v,w,m=safe_command(None,None,.61,0,scan(f=.55,fl=.8,fr=.8),self.cfg)
        self.assertTrue(m.startswith('YIELD'))
        self.assertLess(w,0)

if __name__=='__main__':unittest.main()
