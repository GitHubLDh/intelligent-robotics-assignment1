"""Regression: contested follow slots and the head-on re-approach loop.

Two failures observed in Webots:
  1. R5 locked onto R3's green back while R4 was already following R3, and the
     two converged on the same point and collided sideways. Nothing checked
     whether a green back already had a follower.
  2. Robots met head-on, backed off, then drove straight back into each other,
     because the yield rules switch off outside head_on_range and BASE_CURVE
     immediately re-aimed at the other robot.

Isolated logic tests, not Webots collision validation.
"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'controllers/udp_diff_driver'))
sys.path.insert(0, str(ROOT / 'ros2_controller'))
from perception import FOLLOW_SLOT_STANDOFF_M, scan_markers
from safe_logic import SafeParameters, safe_command

OPEN = [2.5] * 360
# Body-to-body spacing of a correctly formed pair: the follower's centre sits
# FOLLOW_SLOT_STANDOFF_M out from the leader's rear marker, which is itself
# 0.126m behind the leader's centre.
PAIR_SPACING = FOLLOW_SLOT_STANDOFF_M + 0.126


class Obj:
    def __init__(self, model, x, y, z=0):
        self.model, self.position = model, (x, y, z)

    def getModel(self):
        return self.model

    def getPosition(self):
        return self.position


def robot(x, y, heading):
    """A body plus its two markers, as the recognition API reports them."""
    c, s = math.cos(heading), math.sin(heading)
    return [Obj('simple_diff_robot', x, y),
            Obj('front_marker', x + 0.126 * c, y + 0.126 * s),
            Obj('rear_marker', x - 0.126 * c, y - 0.126 * s)]


class ContestedSlotTests(unittest.TestCase):
    """A green back with a robot already in its slot must not be adopted."""

    def test_occupied_slot_retargets_to_the_occupant(self):
        # The observed failure. We see R3 tail-on so its green back is
        # classifiable, while R4 - already in R3's slot but still swinging
        # into alignment - is broadside to us, so NEITHER of its markers
        # projects closer and perception cannot classify it. Its body is
        # still detected, and that is what makes the slot test possible.
        leader = robot(1.10, 0.0, 0.0)
        slot_x = 1.10 - 0.126 - FOLLOW_SLOT_STANDOFF_M
        occupant = robot(slot_x, 0.0, math.pi / 2)
        out = scan_markers(leader + occupant)

        self.assertEqual(out['rear_count'], 1, 'only the leader is classifiable')
        self.assertEqual(out['uncertain_count'], 1, 'occupant is broadside')
        self.assertEqual(out['body_count'], 2, 'but its body is detected')
        # Target must be the occupant's body, not the leader's marker.
        self.assertAlmostEqual(out['rear'][0], slot_x, places=6)
        self.assertLess(out['rear'][0], 1.10 - 0.126,
                        'must not aim at the contested leader')

    def test_free_slot_is_followed_normally(self):
        out = scan_markers(robot(1.10, 0.0, 0.0))
        self.assertEqual(out['body_count'], 1)
        self.assertAlmostEqual(out['rear'][0], 1.10 - 0.126, places=6)

    def test_tail_on_approach_still_picks_the_nearest_tail(self):
        # Approaching a formed pair from behind: the tail is already nearest,
        # so behaviour must be unchanged.
        leader = robot(1.50, 0.0, 0.0)
        tail = robot(1.50 - PAIR_SPACING, 0.0, 0.0)
        out = scan_markers(leader + tail)
        self.assertAlmostEqual(out['rear'][0],
                               1.50 - PAIR_SPACING - 0.126, places=6)

    def test_never_returns_no_target_when_a_green_is_visible(self):
        # Losing a slot must never mean "no target": that is what sent robots
        # wandering off and left the separate trains stranded apart.
        leader = robot(1.10, 0.0, 0.0)
        slot_x = 1.10 - 0.126 - FOLLOW_SLOT_STANDOFF_M
        for heading in (0.0, math.pi / 2, math.pi, -math.pi / 2, 0.8):
            out = scan_markers(leader + robot(slot_x, 0.0, heading))
            self.assertIsNotNone(out['rear'], heading)

    def test_robot_beside_the_slot_does_not_count_as_occupying_it(self):
        # A robot merely passing nearby must not make us abandon a valid
        # leader, or robots would refuse to follow anything in open traffic.
        leader = robot(1.10, 0.0, 0.0)
        bystander = robot(1.10 - 0.126 - FOLLOW_SLOT_STANDOFF_M, 0.75, 0.0)
        out = scan_markers(leader + bystander)
        self.assertAlmostEqual(out['rear'][0], 1.10 - 0.126, places=6)


class EarlyPassingTests(unittest.TestCase):
    """Right-hand passing must engage before a head-on can develop."""

    def test_oncoming_biases_right_while_still_moving(self):
        v, w, mode = safe_command(None, None, 1.20, 0.05, OPEN, SafeParameters())
        self.assertEqual(mode, 'PASS_RIGHT')
        self.assertGreater(v, 0.0, 'an arc separates; a pivot does not')
        self.assertLess(w, 0.0, 'both robots turn right, so they diverge')

    def test_backing_off_does_not_re_aim(self):
        # The exact loop: just outside head_on_range the old controller fell
        # through to BASE_CURVE and drove straight back in.
        for d in (0.72, 0.85, 1.10, 1.50, 1.75):
            v, w, mode = safe_command(None, None, d, 0.0, OPEN, SafeParameters())
            self.assertEqual(mode, 'PASS_RIGHT', d)
            self.assertLess(w, 0.0, d)

    def test_bias_strengthens_as_gap_closes(self):
        near = safe_command(None, None, 0.80, 0.0, OPEN, SafeParameters())[1]
        far = safe_command(None, None, 1.70, 0.0, OPEN, SafeParameters())[1]
        self.assertLess(near, far, 'closer means a firmer turn')

    def test_bias_stays_gentle_enough_to_keep_our_own_follower(self):
        # A robot that swings hard drops out of its follower's ~50 deg camera
        # cone and breaks a train that had already formed.
        worst = min(safe_command(None, None, d, 0.0, OPEN, SafeParameters())[1]
                    for d in (0.71, 0.9, 1.2, 1.5, 1.79))
        self.assertLess(abs(worst) * 1.0, math.radians(50))

    def test_head_on_never_creeps_forward(self):
        v, w, mode = safe_command(None, None, .60, 0, OPEN, SafeParameters())
        self.assertEqual(mode, 'YIELD')
        self.assertEqual(v, 0.0)
        self.assertLess(w, 0.0)

    def test_open_scene_without_markers_unchanged(self):
        v, w, mode = safe_command(
            None, None, None, None, OPEN,
            SafeParameters(forward_speed=.14, base_angular_speed=.24))
        self.assertEqual((v, w, mode), (.14, .24, 'BASE_CURVE'))


class RivalVisibilityTests(unittest.TestCase):
    """The leader LiDAR mask must not erase a rival near the same slot."""

    def test_rival_nearer_than_leader_is_not_masked(self):
        from safe_logic import lidar_sectors
        scan = [2.5] * 360
        for i in range(173, 188):
            scan[i] = 0.62          # 14cm nearer than the green mark
        sec = lidar_sectors(scan, rear_distance=0.76, rear_bearing=0.0)
        self.assertAlmostEqual(sec['front'], 0.62)

    def test_leader_itself_is_still_masked(self):
        from safe_logic import lidar_sectors
        scan = [2.5] * 360
        for i in range(173, 188):
            scan[i] = 0.86          # leader body: marker reading + ~0.13m
        sec = lidar_sectors(scan, rear_distance=0.72, rear_bearing=0.0)
        self.assertEqual(sec['front'], 2.5)


if __name__ == '__main__':
    unittest.main()
