"""Camera classification and safer LiDAR wall-response logic tests.

No ROS 2/Webots required. Real camera occlusion/physics remain user tests.
"""
import sys
import unittest
import math
from pathlib import Path
root = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_controller'))
sys.path.insert(0,str(root/'controllers/udp_diff_driver'))
from safe_logic import SafeParameters, safe_command
from perception import scan_markers

class MockObject:
    def __init__(self,model,x,y): self.model,self.x,self.y=model,x,y
    def getModel(self): return self.model
    def getPosition(self): return (self.x,self.y,0.0)

class CameraTests(unittest.TestCase):
    def test_green_rear(self):
        r=scan_markers([MockObject('simple_diff_robot',1.12,0), MockObject('rear_marker',1.0,0)])
        self.assertIsNotNone(r['rear'])
        self.assertIsNone(r['front'])
        self.assertEqual(r['raw_count'],2)
        self.assertEqual(r['model_types'],['rear_marker','simple_diff_robot'])
    def test_red_only_is_seen_but_not_target(self):
        r=scan_markers([MockObject('simple_diff_robot',1.12,0), MockObject('front_marker',1.0,0)])
        self.assertIsNone(r['rear'])
        self.assertIsNotNone(r['front'])
        self.assertEqual(r['count'],1)
    def test_unknown_model_is_diagnostic_only(self):
        r=scan_markers([MockObject('simple_diff_robot',.8,0)])
        self.assertEqual(r['count'],0)
        self.assertEqual(r['raw_count'],1)
        self.assertEqual(r['model_types'],['simple_diff_robot'])
    def test_cannot_see_object_behind_camera(self):
        r=scan_markers([MockObject('rear_marker',-1,0)])
        self.assertIsNone(r['rear'])

class WallTests(unittest.TestCase):
    cfg=SafeParameters()
    @staticmethod
    def scan(front, left=2.5, right=2.5, rear=2.5):
        out=[2.5]*360
        for i in range(360):
            a=abs(((i-180+180)%360)-180)
            if a<=18:out[i]=front
            if 70<=i-180<=110:out[i]=left
            if -110<=i-180<=-70:out[i]=right
            if abs(abs(i-180)-180)<=25:out[i]=rear
        return out
    def test_front_wall_turns_if_pivot_clear(self):
        v,w,mode=safe_command(None,None,None,None,self.scan(.26),self.cfg)
        self.assertEqual(mode,'BRAKE_TURN')
        self.assertEqual(v,0)
        self.assertLess(w,0)
    def test_moderate_wall_turns_not_reverses(self):
        v,w,mode=safe_command(None,None,None,None,self.scan(.32),self.cfg)
        self.assertLessEqual(v,0)
        self.assertGreaterEqual(v,0)
        self.assertLess(w,0)
        self.assertIn(mode,('AVOID_FRONT','CORNER_CLEAR_TURN','CLEAR_FORWARD_TURN'))
    def test_backoff_when_pivot_blocked_and_rear_clear(self):
        v,w,mode=safe_command(None,None,None,None,self.scan(.24,left=.15,right=.16),self.cfg)
        self.assertLess(v,0)
        self.assertEqual(mode,'EMERGENCY_BACKOFF')
    def test_no_backoff_into_obstacle(self):
        v,w,mode=safe_command(None,None,None,None,self.scan(.24,left=.15,right=.16,rear=.15),self.cfg)
        self.assertEqual(mode,'BLOCKED_STOP')
        self.assertEqual((v,w),(0,0))
    def test_headon_priority_remains(self):
        v,w,mode=safe_command(None,None,.65,0,self.scan(.50),self.cfg)
        self.assertTrue(mode.startswith('YIELD'))
        self.assertLess(w,0)

if __name__=='__main__': unittest.main()
