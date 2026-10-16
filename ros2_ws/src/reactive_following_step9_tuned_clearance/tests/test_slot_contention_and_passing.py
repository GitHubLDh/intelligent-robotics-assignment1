"""Regression: contested follow slots and the head-on re-approach loop.

Two failures this pins down, both observed in Webots:
  1. Two robots converging on the same green back collided sideways, because
     nothing checked whether that back already had a follower.
  2. Robots met head-on, backed off, then drove straight back into each other
     because the yield rules switch off outside head_on_range and BASE_CURVE
     immediately re-aimed at the other robot.

Isolated logic tests on safe_command, not Webots collision validation.
"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_controller'))
from safe_logic import SafeParameters, marker_gap, safe_command

OPEN = [2.5] * 360


def cfg(**kw):
    return SafeParameters(**kw)


class MarkerGapTests(unittest.TestCase):
    def test_gap_is_symmetric_and_metric(self):
        # Same bearing: gap is the plain range difference.
        self.assertAlmostEqual(marker_gap(1.0, 0.0, 0.6, 0.0), 0.4)
        # Same range, opposite bearings: isoceles chord.
        self.assertAlmostEqual(marker_gap(1.0, 0.3, 1.0, -0.3),
                               2 * math.sin(0.3), places=6)
        self.assertAlmostEqual(marker_gap(1.2, .2, .7, -.1),
                               marker_gap(.7, -.1, 1.2, .2))

    def test_gap_never_negative_under_rounding(self):
        self.assertGreaterEqual(marker_gap(1.0, 0.0, 1.0, 0.0), 0.0)


class ContestedSlotTests(unittest.TestCase):
    """A red front marker next to a green back means that back is taken."""

    def test_angled_approach_queues_instead_of_contesting(self):
        # Leader's green back off to our right; its existing follower's red
        # front marker just to the left of it. This is the geometry that
        # produced sideways collisions: the occupant is NOT between us and the
        # leader, so an occlusion test cannot see it.
        v, w, mode = safe_command(1.0, -0.25, 1.0, 0.10, OPEN, cfg())
        self.assertEqual(mode, 'QUEUE_APPROACH')
        self.assertGreater(v, 0.0, 'must keep closing on the group')
        self.assertGreater(w, 0.0, 'swing to the occupant far side from leader')

    def test_queue_swings_around_occupant_when_close(self):
        v, w, mode = safe_command(0.70, -0.30, 0.50, 0.05, OPEN, cfg())
        self.assertEqual(mode, 'QUEUE_SWING')
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)

    def test_swing_direction_mirrors(self):
        # Occupant on the leader's right instead: swing right.
        v, w, mode = safe_command(0.70, 0.30, 0.50, -0.05, OPEN, cfg())
        self.assertEqual(mode, 'QUEUE_SWING')
        self.assertLess(w, 0.0)

    def test_occupant_red_marker_is_not_treated_as_head_on(self):
        # A follower's red marker points at its own leader, not at us.
        for d_red in (0.30, 0.45, 0.60):
            mode = safe_command(0.70, -0.30, d_red, 0.05, OPEN, cfg())[2]
            self.assertFalse(mode.startswith('YIELD'), (d_red, mode))

    def test_queueing_never_abandons_the_group(self):
        # Losing a slot must not fall back to BASE_CURVE and wander off:
        # that left the separate trains on opposite sides of the map.
        for d_green, b_green in ((1.4, -.2), (1.0, -.25), (.8, -.3)):
            v, w, mode = safe_command(d_green, b_green, d_green - .1, .05,
                                      OPEN, cfg())
            self.assertTrue(mode.startswith('QUEUE'), (d_green, mode))
            self.assertNotIn('BASE', mode)

    def test_uncontested_leader_still_followed(self):
        # No red marker: ordinary following must be untouched.
        self.assertEqual(safe_command(1.0, 0.0, None, None, OPEN, cfg())[2],
                         'FOLLOW')
        self.assertEqual(safe_command(.29, .06, None, None, OPEN, cfg()),
                         (0, 0, 'FOLLOW_HOLD'))

    def test_far_apart_markers_are_not_a_taken_slot(self):
        # Green back near, unrelated red robot well away from it: that is a
        # genuine oncoming robot, not a slot occupant.
        self.assertFalse(
            marker_gap(1.0, -0.4, 1.0, 0.4) < cfg().slot_taken_range)


class EarlyPassingTests(unittest.TestCase):
    """Right-hand passing must engage before a head-on can develop."""

    def test_oncoming_biases_right_while_still_moving(self):
        v, w, mode = safe_command(None, None, 1.20, 0.05, OPEN, cfg())
        self.assertEqual(mode, 'PASS_RIGHT')
        self.assertGreater(v, 0.0, 'an arc separates; a pivot does not')
        self.assertLess(w, 0.0, 'both robots turn right, so they diverge')

    def test_backing_off_does_not_re_aim(self):
        # The exact loop: just outside head_on_range the old controller fell
        # through to BASE_CURVE and drove straight back in.
        for d in (0.72, 0.85, 1.10, 1.50, 1.75):
            v, w, mode = safe_command(None, None, d, 0.0, OPEN, cfg())
            self.assertEqual(mode, 'PASS_RIGHT', d)
            self.assertLess(w, 0.0, d)

    def test_bias_strengthens_as_gap_closes(self):
        near = safe_command(None, None, 0.80, 0.0, OPEN, cfg())[1]
        far = safe_command(None, None, 1.70, 0.0, OPEN, cfg())[1]
        self.assertLess(near, far, 'closer means a firmer turn')

    def test_bias_stays_gentle_enough_to_keep_our_own_follower(self):
        # A robot that swings hard drops out of its follower's ~50 deg camera
        # cone and breaks a train that had already formed. Over a full second
        # of worst-case bias the heading change must stay inside that cone.
        worst = min(safe_command(None, None, d, 0.0, OPEN, cfg())[1]
                    for d in (0.71, 0.9, 1.2, 1.5, 1.79))
        self.assertLess(abs(worst) * 1.0, math.radians(50))

    def test_head_on_never_creeps_forward(self):
        # Inside head_on_range we must not advance toward the other robot.
        v, w, mode = safe_command(None, None, .60, 0, OPEN, cfg())
        self.assertEqual(mode, 'YIELD')
        self.assertEqual(v, 0.0)
        self.assertLess(w, 0.0)

    def test_open_scene_without_markers_unchanged(self):
        v, w, mode = safe_command(None, None, None, None, OPEN,
                                  cfg(forward_speed=.14, base_angular_speed=.24))
        self.assertEqual((v, w, mode), (.14, .24, 'BASE_CURVE'))


class RivalVisibilityTests(unittest.TestCase):
    """The leader mask must not erase a rival converging on the same slot."""

    def test_rival_nearer_than_leader_is_not_masked(self):
        from safe_logic import lidar_sectors
        scan = [2.5] * 360
        for i in range(173, 188):
            scan[i] = 0.62          # rival, 14cm nearer than the green mark
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
