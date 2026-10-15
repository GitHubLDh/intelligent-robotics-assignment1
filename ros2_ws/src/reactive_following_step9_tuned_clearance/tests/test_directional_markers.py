"""Offline adversarial tests for local camera marker classification."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'controllers/udp_diff_driver'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_controller'))
from perception import scan_markers

class Obj:
    def __init__(self, model, x, y, z=0): self.model, self.position = model,(x,y,z)
    def getModel(self): return self.model
    def getPosition(self): return self.position

def robot(x, y, orientation, include_front=True, include_rear=True):
    # orientation is local heading about Z, generated for synthetic tests only.
    # The classifier receives only current camera-relative coordinates.
    c,s = math.cos(orientation), math.sin(orientation)
    o=[Obj('simple_diff_robot',x,y)]
    if include_front: o.append(Obj('front_marker',x+0.126*c,y+0.126*s))
    if include_rear: o.append(Obj('rear_marker',x-0.126*c,y-0.126*s))
    return o

class MarkerClassification(unittest.TestCase):
    def test_head_on_observation_causes_yield_not_follow(self):
        from safe_logic import safe_command, SafeParameters
        # Test uses synthetic valid LiDAR scan while red/green are recognized.
        x=scan_markers(robot(.65,0,math.pi))
        scan=[2.5]*360
        for idx in range(167,194): scan[idx]=.55
        rear=x['rear'] or (None,None)
        front=x['front'] or (None,None)
        v,w,mode=safe_command(*rear,*front,scan,SafeParameters())
        self.assertTrue(mode.startswith('YIELD'), (v,w,mode))
    def test_rear_observation_causes_follow_not_yield(self):
        from safe_logic import safe_command, SafeParameters
        x=scan_markers(robot(1.45,0,0))
        rear=x['rear'] or (None,None)
        front=x['front'] or (None,None)
        v,w,mode=safe_command(*rear,*front,[2.5]*360,SafeParameters())
        self.assertEqual(mode,'FOLLOW')
    def test_front_on_both_markers_recognized_rejects_rear(self):
        r=scan_markers(robot(1.5,0,math.pi))
        self.assertIsNone(r['rear']); self.assertIsNotNone(r['front'])
        self.assertEqual((r['front_count'],r['rear_count']),(1,0))
        self.assertEqual(r['raw_count'],3)
        self.assertIn('rear_marker',r['model_types']) # unfiltered raw observation!
    def test_rear_on_both_recognized_accepts_rear(self):
        r=scan_markers(robot(1.5,0,0))
        self.assertIsNone(r['front']); self.assertIsNotNone(r['rear'])
        self.assertEqual((r['front_count'],r['rear_count']),(0,1))
    def test_front_only_near_surface(self):
        r=scan_markers(robot(1.5,0,math.pi,include_rear=False))
        self.assertIsNotNone(r['front']); self.assertIsNone(r['rear'])
    def test_rear_only_near_surface(self):
        r=scan_markers(robot(1.5,0,0,include_front=False))
        self.assertIsNotNone(r['rear']); self.assertIsNone(r['front'])
    def test_single_wrong_side_marker_is_rejected(self):
        r=scan_markers(robot(1.5,0,math.pi,include_front=False))
        self.assertIsNone(r['rear']); self.assertIsNone(r['front'])
    def test_side_on_is_ambiguous_and_not_followable(self):
        r=scan_markers(robot(1.5,0,math.pi/2))
        self.assertIsNone(r['rear']); self.assertIsNone(r['front'])
        self.assertEqual(r['uncertain_count'],1)
    def test_two_robots_front_and_rear_independent(self):
        objs=robot(1.2,-.55,math.pi)+robot(1.65,.65,0)
        r=scan_markers(objs)
        self.assertEqual((r['front_count'],r['rear_count']),(1,1))
        self.assertIsNotNone(r['rear']); self.assertIsNotNone(r['front'])
    def test_no_body_fail_closed(self):
        r=scan_markers([Obj('rear_marker',1.2,0),Obj('front_marker',1.5,0)])
        self.assertIsNone(r['rear']); self.assertIsNone(r['front'])
        self.assertEqual(r['unmatched_count'],2)
    def test_ambiguous_nearby_bodies_reject(self):
        objs=[Obj('simple_diff_robot',1.2,-.035),Obj('simple_diff_robot',1.2,.035),Obj('rear_marker',1.1,0)]
        r=scan_markers(objs)
        self.assertIsNone(r['rear']); self.assertEqual(r['unmatched_count'],1)
    def test_outside_field_of_view_rejected(self):
        r=scan_markers(robot(1.5,2.2,0))
        self.assertIsNone(r['rear'])
    def test_invalid_position_fail_closed(self):
        r=scan_markers([Obj('simple_diff_robot',1,0),Obj('rear_marker',float('nan'),0)])
        self.assertIsNone(r['rear'])
    def test_nearest_accepted_rear_selected(self):
        objs=robot(1.4,-.5,0)+robot(2.0,.6,0)
        r=scan_markers(objs)
        self.assertGreaterEqual(r['rear_count'],1)
        self.assertLess(r['rear'][0],2.0)

if __name__=='__main__': unittest.main()
