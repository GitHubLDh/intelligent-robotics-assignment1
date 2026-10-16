"""Basic reactive following. Four rules, checked most-urgent first.

  1. red NEARER THAN GREEN and very close -> turn until the coast ahead is
                                             clear, then drive that way
  2. red NEARER THAN GREEN, within red_turn_distance -> turn right
  3. green closer than follow_distance               -> follow that robot
  4. something detected beyond approach_distance     -> go to it
  otherwise                                          -> base curve

Rules 1 and 2 compare the two colours rather than just checking that red is
visible. Seeing red at all is not a threat - every robot we follow has a red
front half somewhere. What matters is which colour is NEARER: if the green
back is the closer thing, the robot in front of us is facing away and we keep
following it.

Red is the front half of a robot (it is facing us), green is the rear half
(it is facing away, so it can be followed). Sensing is frontal only: a 100
degree, 4 m camera and LiDAR. One current snapshot per decision - no target
memory, no global pose, no inter-robot communication.

Linear speed is never zero in any rule, so no arrangement of robots can
settle into a standstill.
"""
from dataclasses import dataclass
import math

N_BEAMS = 360
FIELD_OF_VIEW = math.radians(100)
HALF_FOV = FIELD_OF_VIEW / 2.0
RANGE_MAX = 4.0


def clamp(v, lo, hi):
    return min(hi, max(lo, v))


@dataclass(frozen=True)
class FollowParameters:
    forward_speed: float = 0.15
    # Never zero: a robot that stops can deadlock a whole chain behind it.
    min_speed: float = 0.03
    steering_gain: float = 1.5
    max_angular_speed: float = 1.1
    base_angular_speed: float = -0.20
    turn_speed: float = 0.80
    # Rule 4: go towards anything seen further away than this.
    approach_distance: float = 1.00
    # Rule 3: a green back nearer than this is followed.
    follow_distance: float = 1.20
    # Standoff held while following, so we close in but do not ram.
    desired_distance: float = 0.45
    # Tightest arc allowed while FOLLOWING. Steering straight at the leader is
    # pure pursuit: it cuts inside the leader's path, so a ring of followers
    # spirals inward until they pile up. In the 0.95 m start ring the leader
    # sits at ~0.63 rad, giving w = 1.5*0.63 = 0.94 rad/s and a turn radius of
    # only v/w = 0.15 m. Capping by RADIUS rather than by a fixed w matters:
    # the robots run at different speeds (0.12-0.16), so one shared w cap
    # would give each a different radius and distort the ring, whereas
    # w_cap = forward_speed / min_follow_radius holds them all on one circle.
    # Only FOLLOW is capped; APPROACH still turns at full rate to acquire.
    min_follow_radius: float = 0.90
    # Rule 2: a robot facing us nearer than this makes us turn right.
    red_turn_distance: float = 0.90
    # Rule 1: red this close is an emergency.
    very_close_distance: float = 0.45
    # Ahead must beat this for the coast to count as clear.
    coast_clear_range: float = 1.00
    # Half-width of the cone that has to be free of red before we call the
    # coast clear and drive off. The LiDAR sits above the other robots'
    # bodies and cannot see a peer closer than ~0.75 m, so in a nose-to-nose
    # contact it reports 4 m of open space; the camera is the only sensor
    # that can see the robot we are touching.
    coast_cone: float = 0.60
    # Acceleration limits, applied by the node.
    max_angular_accel: float = 2.5
    max_linear_accel: float = 0.60


def visible(distance, bearing):
    return (distance is not None and bearing is not None and
            math.isfinite(distance) and math.isfinite(bearing) and
            0.0 < distance <= RANGE_MAX and abs(bearing) <= HALF_FOV)


def beam_angle(index, beams=N_BEAMS):
    """Index 0 is the right edge (-50 deg), 180 ahead, 359 the left edge."""
    return -HALF_FOV + FIELD_OF_VIEW * index / (beams - 1)


def clearance_ahead(scan, half_width=math.radians(15)):
    """Nearest LiDAR return in the forward cone, or None if the scan is bad."""
    if (scan is None or len(scan) != N_BEAMS or
            not all(isinstance(d, (int, float)) and math.isfinite(d) and
                    0 < d <= RANGE_MAX + 0.01 for d in scan)):
        return None
    ahead = [d for i, d in enumerate(scan) if abs(beam_angle(i)) <= half_width]
    return min(ahead) if ahead else RANGE_MAX


def follow_command(green_distance, green_bearing,
                   red_distance, red_bearing, ranges, cfg):
    """Return (linear m/s, angular rad/s, mode) for the current snapshot."""
    ahead = clearance_ahead(ranges)
    if ahead is None:
        return 0.0, 0.0, 'SENSOR_FAILSAFE'

    green = visible(green_distance, green_bearing)
    red = visible(red_distance, red_bearing)
    right = -min(cfg.turn_speed, cfg.max_angular_speed)

    # Red alone means nothing - every robot we follow has a red front half
    # somewhere in view. It is a threat only when it is the NEARER colour,
    # i.e. the thing closest to us is facing us. While a green back is the
    # closer thing, we keep following it.
    red_threatens = red and (not green or red_distance < green_distance)

    # 1. Facing us and very close. Turn until the way ahead is actually
    #    clear, then take it. Both robots turn right, so a head-on pair
    #    separates instead of mirroring each other.
    #
    #    "Clear" must mean clear to BOTH sensors. Checking only the LiDAR
    #    made a nose-to-nose pair drive into each other at full speed
    #    forever: a robot that close is invisible to the LiDAR, which
    #    reported 4 m of open road. Rotating is what resolves it - our own
    #    heading changes, so the red slides out of the forward cone even
    #    though neither robot has moved - and then we leave along the gap.
    red_in_cone = (red and red_distance < cfg.red_turn_distance
                   and abs(red_bearing) <= cfg.coast_cone)
    if red_threatens and red_distance < cfg.very_close_distance:
        if ahead < cfg.coast_clear_range or red_in_cone:
            # No forward component: we are touching something, so creeping
            # ahead only pushes. Rotation alone clears the cone, and it is
            # bounded - roughly coast_cone / turn_speed of a second.
            return 0.0, right, 'ESCAPE_TURN'
        return cfg.forward_speed, 0.0, 'ESCAPE_GO'

    # 2. Facing us, close but not yet critical: turn right.
    if red_threatens and red_distance < cfg.red_turn_distance:
        return cfg.forward_speed, right, 'AVOID_RED'

    # 3. A green back within following range: steer onto it, easing off so we
    #    settle near desired_distance rather than driving into it. The speed
    #    floor means a follower never stops, so a chain cannot freeze.
    if green and green_distance < cfg.follow_distance:
        follow_cap = min(cfg.max_angular_speed,
                         cfg.forward_speed / max(cfg.min_follow_radius, 1e-6))
        turn = clamp(cfg.steering_gain * green_bearing,
                     -follow_cap, follow_cap)
        reach = max(cfg.follow_distance - cfg.desired_distance, 1e-6)
        ratio = clamp((green_distance - cfg.desired_distance) / reach, 0.0, 1.0)
        speed = cfg.min_speed + (cfg.forward_speed - cfg.min_speed) * ratio
        return speed, turn, 'FOLLOW'

    # 4. Something is out there, further off: head towards it.
    candidates = [(green_distance, green_bearing)] if green else []
    if red:
        candidates.append((red_distance, red_bearing))
    if candidates:
        distance, bearing = min(candidates)
        if distance >= cfg.approach_distance:
            turn = clamp(cfg.steering_gain * bearing,
                         -cfg.max_angular_speed, cfg.max_angular_speed)
            return cfg.forward_speed, turn, 'APPROACH'

    # Nothing to act on: wander on the base curve.
    return cfg.forward_speed, clamp(cfg.base_angular_speed,
                                    -cfg.max_angular_speed,
                                    cfg.max_angular_speed), 'BASE_CURVE'
