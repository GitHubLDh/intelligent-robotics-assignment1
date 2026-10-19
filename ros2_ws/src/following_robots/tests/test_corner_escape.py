"""Stateless corner-escape regression tests (approximate LiDAR geometry)."""
import math
import sys
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_controller'))
from safe_logic import SafeParameters, safe_command


def sector_scan(x, y, heading):
    """Ideal rays against 6-m square perimeter (not a Webots physics test)."""
    result=[]
    for i in range(360):
        angle=heading-math.pi+2*math.pi*i/360
        vx,vy=math.cos(angle), math.sin(angle)
        hits=[]
        if vx>1e-9: hits.append((2.88-x)/vx)
        if vx< -1e-9: hits.append((-2.88-x)/vx)
        if vy>1e-9: hits.append((2.88-y)/vy)
        if vy< -1e-9: hits.append((-2.88-y)/vy)
        result.append(min(2.5, min(t for t in hits if t>0)))
    return result


class CornerEscapeTests(unittest.TestCase):
    cfg=SafeParameters(forward_speed=.14,base_angular_speed=.24)

    def test_right_turn_does_not_flip_due_to_asymmetry(self):
        # Small perturbations in left/right clearances should not alternate
        # escape direction while frontal obstruction remains.
        a=math.pi/4
        for shift in [-.05, -.025, 0, .025, .05]:
            scan=sector_scan(2.36+shift, 2.37-shift, a)
            v,w,mode=safe_command(None,None,None,None,scan,self.cfg)
            self.assertEqual(mode,'CORNER_CLEAR_TURN')
            self.assertEqual(v,0.0)
            self.assertLess(w,0.0)

    def test_stays_committed_to_clearing_front(self):
        # At intermediate headings the old controller alternated BASE_CURVE
        # (+yaw) with AVOID_FRONT (-yaw), and could spin without escaping.
        for degrees in [45,25,0,-25,-45,-65]:
            scan=sector_scan(2.36,2.37,math.radians(degrees))
            v,w,mode=safe_command(None,None,None,None,scan,self.cfg)
            self.assertGreaterEqual(v,0.0,(degrees,mode))
            self.assertLessEqual(v,.075,(degrees,mode))
            self.assertLess(w,0.0,(degrees,mode))
            self.assertIn(mode,('CORNER_CLEAR_TURN','CLEAR_FORWARD_TURN','CLEAR_FORWARD_ARC','AVOID_FRONT'))

    def test_escape_open_space_resumes_base(self):
        v,w,mode=safe_command(None,None,None,None,
                               sector_scan(2.36,2.37,-math.pi/2),self.cfg)
        self.assertIn(mode, ('BASE_CURVE','BASE_CLEAR_BLEND'))
        self.assertGreater(v,0.0)

    def test_simplified_corner_escape_moves_clear(self):
        x,y,angle=2.36,2.37,math.pi/4
        saw_turn=False
        for _ in range(160): # 8 seconds at 50ms: ideal kinematics
            v,w,mode=safe_command(None,None,None,None,sector_scan(x,y,angle),self.cfg)
            saw_turn |= mode in ('CORNER_CLEAR_TURN','CLEAR_FORWARD_TURN')
            x+=v*math.cos(angle)*.05
            y+=v*math.sin(angle)*.05
            angle+=w*.05
        self.assertTrue(saw_turn)
        self.assertLess(y,2.0) # progress away from northeast corner
        self.assertLess(x,2.7) # avoid east wall
        self.assertLess(y,2.7) # avoid north wall

    def test_green_hold_and_headon_still_prioritized(self):
        scan=sector_scan(2.36,2.37,math.pi/4)
        # Camera green marker geometry is inconsistent with corner wall,
        # so the controller must not pretend it is safe to HOLD.
        self.assertNotEqual(safe_command(2.0,0,None,None,scan,self.cfg)[2], 'FOLLOW_HOLD')
        # A red marker with measured distance <0.7 triggers yielding.
        _,_,mode=safe_command(None,None,.6,0,scan,self.cfg)
        self.assertTrue(mode.startswith('YIELD') or mode.startswith('BRAKE'),mode)


if __name__ == '__main__': unittest.main()
