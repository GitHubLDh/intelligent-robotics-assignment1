"""Tests that don't require ROS 2/Webots; real sensor geometry needs Webots."""
import json
import math
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
for p in [ROOT / 'controllers' / 'udp_diff_driver', ROOT / 'ros2_bridge', ROOT / 'ros2_controller']:
    sys.path.insert(0, str(p))
from lidar_scan import combine_layers
from sensor_packet import sensor_packet
from sensor_snapshot import parse_snapshot
from safe_logic import SafeParameters, lidar_sectors, safe_command, beam_angle


def synthetic(default=2.5, **points):
    out = [default] * 360
    for idx, value in points.items():
        out[int(idx)] = value
    return out


def obstacle(angle, distance, width=14):
    scan = synthetic()
    mid = round(180 + math.degrees(angle)) % 360
    for idx in range(mid-width//2, mid+width//2+1):
        scan[idx % 360] = distance
    return scan


class LidarLayerTests(unittest.TestCase):
    def test_layers_min(self):
        arr = [2.5] * 1080
        arr[180] = 0.8
        arr[360+180] = 0.6
        arr[720+180] = 1.0
        result = combine_layers(arr)
        self.assertEqual(len(result), 360)
        self.assertEqual(result[180], 0.6)
    def test_inf_clear(self):
        self.assertEqual(combine_layers([float('inf')]*1080)[180], 2.5)
    def test_invalid_not_clear(self):
        self.assertIsNone(combine_layers([]))
        self.assertIsNone(combine_layers([float('nan')]*1080))
    def test_single_nan_beam_closed(self):
        v = [2.5]*1080
        for layer in range(3):
            v[layer*360+180] = math.nan
        self.assertEqual(combine_layers(v)[180], 0.0)


class SnapshotTests(unittest.TestCase):
    def test_roundtrip_with_target(self):
        msg = sensor_packet({'visible':True,'distance_m':1.3,'bearing_rad':.2,
                             'front_visible':False,'lidar_ranges_m':[2.5]*360})
        self.assertEqual(len(msg),364)
        parsed = parse_snapshot(msg)
        self.assertAlmostEqual(parsed[0],1.3)
        self.assertIsNone(parsed[2])
        self.assertEqual(len(parsed[4]),360)
    def test_fails_closed_on_missing_scan(self):
        self.assertIsNone(parse_snapshot(sensor_packet({})))
    def test_reject_truncated(self):
        self.assertIsNone(parse_snapshot([2.5]*10))
    def test_reject_bad_range(self):
        x=[float('nan')]*4+[2.5]*360
        x[184]=-1
        self.assertIsNone(parse_snapshot(x))


class SectorTests(unittest.TestCase):
    def test_scan_axis_convention(self):
        self.assertAlmostEqual(beam_angle(180),0)
        self.assertAlmostEqual(beam_angle(90),-math.pi/2)
        self.assertAlmostEqual(beam_angle(270),math.pi/2)
    def test_front_detection(self):
        sec = lidar_sectors(obstacle(0,.60))
        self.assertEqual(sec['front'],.60)
        self.assertEqual(sec['rear'],2.5)
    def test_rear_detection(self):
        sec = lidar_sectors(obstacle(math.pi,.32))
        self.assertEqual(sec['rear'],.32)
        self.assertEqual(sec['front'],2.5)
    def test_side_right_detection(self):
        sec = lidar_sectors(obstacle(-math.pi/2,.24))
        self.assertEqual(sec['right'],.24)
    def test_side_left_detection(self):
        sec = lidar_sectors(obstacle(math.pi/2,.24))
        self.assertEqual(sec['left'],.24)
    def test_only_current_green_target_masked(self):
        sec = lidar_sectors(obstacle(0,.86),rear_distance=.72,rear_bearing=0)
        self.assertEqual(sec['front'],2.5)
        self.assertAlmostEqual(sec['raw_front'],.86)
    def test_nearer_wall_not_hidden(self):
        sec = lidar_sectors(obstacle(0,.28),rear_distance=1.1,rear_bearing=0)
        self.assertEqual(sec['front'],.28)
    def test_distant_wall_not_hidden(self):
        sec = lidar_sectors(obstacle(0,1.1),rear_distance=.55,rear_bearing=0)
        self.assertEqual(sec['front'],1.1)
    def test_invalid_fails_closed(self):
        self.assertIsNone(lidar_sectors([math.nan]*360))
        self.assertIsNone(lidar_sectors([]))


class ControllerTests(unittest.TestCase):
    cfg=SafeParameters()
    def go(self,scan=None,green=None,red=None):
        d,b=green if green else (None,None)
        fd,fb=red if red else (None,None)
        return safe_command(d,b,fd,fb,scan if scan is not None else synthetic(),self.cfg)
    def test_base_curve(self):
        v,w,m=self.go()
        self.assertAlmostEqual(v,self.cfg.forward_speed)
        self.assertEqual(m,'BASE_CURVE')
    def test_no_lidar_failsafe(self):
        self.assertEqual(self.go([float('nan')]*360)[2],'SENSOR_FAILSAFE')
    def test_follow_distant(self):
        v,w,m=self.go(green=(1.6,0.0))
        self.assertEqual(m,'FOLLOW')
        self.assertGreater(v,self.cfg.forward_speed)
    def test_hold_stopped(self):
        self.assertEqual(self.go(green=(.57,.06)),(0,0,'FOLLOW_HOLD'))
    def test_side_wall_avoid(self):
        v,w,m=self.go(obstacle(-math.pi/2,.20))
        self.assertIn(m,('AVOID_SIDE','ESCAPE_SIDE'))
        self.assertEqual(v,0.0)
    def test_front_wall_avoid(self):
        v,w,m=self.go(obstacle(0,.48))
        self.assertIn(m,('AVOID_FRONT','ESCAPE_CORNER','BRAKE_TURN'))
        self.assertLessEqual(v,0)
    def test_rear_blocks_reversing(self):
        scan=obstacle(0,.21)
        for i in range(0,17):
            scan[i]=.22
            scan[359-i]=.22
        v,w,m=self.go(scan)
        self.assertGreaterEqual(v,0)
    def test_corner_can_reverse_when_rear_clear(self):
        scan=obstacle(0,.4)
        for i in range(215,280):
            scan[i]=.3
        v,w,m=self.go(scan)
        self.assertLessEqual(v,0)
    def test_red_avoids_headon(self):
        v,w,m=self.go(obstacle(0,.54),red=(.57,0))
        self.assertTrue(m.startswith(('YIELD','BRAKE','ESCAPE')))
    def test_target_does_not_mask_wall(self):
        v,w,m=self.go(obstacle(0,.28),green=(1.1,0))
        self.assertNotIn(m,('FOLLOW','FOLLOW_ALIGN','FOLLOW_HOLD'))
    def test_close_green_hold_no_swivel(self):
        v,w,m=self.go(obstacle(0,.46),green=(.55,.12))
        self.assertEqual((v,w,m),(0,0,'FOLLOW_HOLD'))
    def test_camera_edge_realign(self):
        v,w,m=self.go(green=(1.5,.79))
        self.assertEqual(m,'FOLLOW_ALIGN')
        self.assertEqual(v,0)
    def test_wall_above_base_slow(self):
        v,w,m=self.go(obstacle(0,.72))
        self.assertEqual(m,'BASE_CLEAR_BLEND')
        self.assertLess(v,self.cfg.forward_speed)


class ConfigTests(unittest.TestCase):
    def test_five_configs_no_legacy_fields(self):
        data=json.loads((ROOT/'config'/'robots.json').read_text())
        self.assertEqual(len(data),5)
        for v in data.values():
            self.assertIn('lidar_slow_range',v)
            self.assertNotIn('sonar_slow_range',v)
            self.assertEqual(v['follow_max_speed'],.21)

if __name__ == '__main__':
    unittest.main()
