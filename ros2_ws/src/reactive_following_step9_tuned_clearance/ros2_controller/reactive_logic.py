"""Basic reactive following. Five rules, checked most-urgent first.

  1. red nearer than green, very close   -> escape (see below)
  2. red nearer than green, within red_turn_distance -> turn right
  3. green visible anywhere                          -> follow that robot
  4. red only, beyond approach_distance              -> go to it
  5. no robot identified, anything inside escape_clear_range -> escape
  otherwise                                          -> base curve

Rules 1 and 5 share one escape: reverse if close enough to be touching,
otherwise rotate on the spot, always the same way, until the way ahead is
clear - then the ordinary rules drive off in whatever direction that turned
out to be. They are deliberately identical because the controller often
cannot tell which case it is in: at contact range the camera detection is
bistable, and a peer's wheel alone reads as an anonymous obstacle.

Rules 1 and 2 compare the two colours rather than just checking that red is
visible. Seeing red at all is not a threat - every robot we follow has a red
front half somewhere. What matters is which colour is NEARER: if the green
back is the closer thing, the robot in front of us is facing away and we keep
following it.

Red is the front half of a robot (it is facing us), green is the rear half
(it is facing away, so it can be followed). Sensing is frontal only: a 100
degree, 4 m camera and LiDAR. One current snapshot per decision - no target
memory, no global pose, no inter-robot communication.

Linear speed is zero or negative only in the escape, where not advancing is
the whole point; every other rule keeps the robot moving, so no arrangement of
them can settle into a standstill.
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
    # Rule 4: go towards anything seen further away than this. Matched to
    # red_turn_distance so a red between the two is not left unhandled.
    approach_distance: float = 0.90
    # Tightest arc allowed while approaching a distant red. Capped like
    # FOLLOW, just less tightly so acquisition stays reasonably quick. Left
    # uncapped, a red at the edge of the cone commanded 1.5*0.83 -> 1.1 rad/s
    # and the robot swung hard enough to hit whatever was beside or behind
    # it; the peak never showed in the log because modes are only logged on
    # change, so it looked like a mild 0.27 right up to the collision.
    approach_min_radius: float = 0.45
    # Speed ramp while following. At or beyond cruise_distance we run at full
    # speed; below it we ease off, reaching the floor at close_distance.
    #
    # The ramp used to reach ZERO at the standoff, which makes a ring settle
    # at the floor speed: every robot closes to the standoff, every robot then
    # commands min_speed, and the whole formation crawls. Ramping down only
    # once we are closer than we want leaves the equilibrium at full speed.
    #
    # cruise_distance must also agree with the ring the turn cap can hold. For
    # five robots, spacing and ring radius are locked: s = 2*R*sin(36 deg) =
    # 1.176*R, and the camera reads 0.19 m less than centre spacing (its own
    # 0.130 offset plus the green half's 0.060). So cruise 0.85 -> spacing
    # 1.04 -> ring R 0.88, which min_follow_radius 0.90 can just hold.
    cruise_distance: float = 0.85
    close_distance: float = 0.50
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
    # Close-range escape, shared by rules 1 and 5. Rotate on the spot while
    # anything sits inside escape_clear_range, and reverse if we are close
    # enough to be touching it. Both rules use the SAME fixed right turn:
    # when they disagreed - rule 1 always right, rule 5 toward the open side -
    # a pair pressed face to face alternated between them and oscillated left
    # and right on the spot, because at contact range the bearing is so
    # sensitive to lateral offset that the camera detection is bistable.
    escape_clear_range: float = 2.00
    contact_range: float = 0.25
    reverse_speed: float = 0.06
    # As wide as the sensor allows. At +/-15 degrees a wall met at an angle
    # was never "ahead" at all, so nothing fired and the robot drove into it
    # and slid along it.
    obstacle_cone: float = 0.785
    # Acceleration limits, applied by the node. Asymmetric on purpose: the
    # limiter exists to stop a target appearing at the edge of the cone from
    # producing an instant full-rate swing, which is an INCREASE in |w|.
    # Unwinding a turn back down to the gentle base curve was never the
    # hazard, and limiting it equally meant a robot entering BASE_CURVE from
    # an avoid turn needed ~8 ticks to reach its base_angular_speed - longer
    # than many base-curve spells last, so it never actually drove the curve.
    max_angular_accel: float = 2.5
    max_angular_decel: float = 6.0
    max_linear_accel: float = 0.60


def visible(distance, bearing):
    return (distance is not None and bearing is not None and
            math.isfinite(distance) and math.isfinite(bearing) and
            0.0 < distance <= RANGE_MAX and abs(bearing) <= HALF_FOV)


def beam_angle(index, beams=N_BEAMS):
    """Index 0 is the right edge (-50 deg), 180 ahead, 359 the left edge."""
    return -HALF_FOV + FIELD_OF_VIEW * index / (beams - 1)


def _valid_scan(scan):
    return (scan is not None and len(scan) == N_BEAMS and
            all(isinstance(d, (int, float)) and math.isfinite(d) and
                0 < d <= RANGE_MAX + 0.01 for d in scan))


def sector_min(scan, low, high):
    """Nearest return with a beam angle in [low, high], else RANGE_MAX."""
    vals = [d for i, d in enumerate(scan) if low <= beam_angle(i) <= high]
    return min(vals) if vals else RANGE_MAX


def clearance_ahead(scan, half_width=0.785):
    """Nearest LiDAR return in the forward cone, or None if the scan is bad.

    half_width is FollowParameters.obstacle_cone when called for real.
    """
    if not _valid_scan(scan):
        return None
    return sector_min(scan, -half_width, half_width)


def follow_command(green_distance, green_bearing,
                   red_distance, red_bearing, ranges, cfg):
    """Return (linear m/s, angular rad/s, mode) for the current snapshot."""
    ahead = clearance_ahead(ranges, cfg.obstacle_cone)
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

    def escape(mode_prefix='ESCAPE'):
        """Shared close-range response: back out if we are touching
        something, otherwise rotate on the spot. Never drives forward - that
        is what made a robot grind along a wall it was supposedly avoiding.
        Always turns the same way, so two robots using it cannot alternate
        against each other. Stateless: the rule simply stops firing once the
        way ahead is clear, and the ordinary rules take over."""
        if ahead < cfg.contact_range:
            return -cfg.reverse_speed, right * 0.5, mode_prefix + '_BACK'
        return 0.0, right, mode_prefix + '_TURN'

    # 1. A robot is facing us and very close: turn on the spot, never drive
    #    forward. There used to be an ESCAPE_GO branch that drove ahead once
    #    the cone looked clear, and it oscillated against ESCAPE_TURN: the
    #    turn pushed the red just outside the cone, GO drove forward, that
    #    brought it back in, and the pair sat there swapping between the two.
    #    While a close red is on screen at all there is no forward direction
    #    we can trust, so we only rotate.
    #
    #    Rotation alone resolves it and is bounded: our own heading changes,
    #    so the red leaves the camera's 50 degree view within about
    #    (0.87 + 0.87) / turn_speed seconds even if neither robot moves. Once
    #    it is gone the ordinary rules drive us off. Both robots turn right,
    #    so a head-on pair separates instead of mirroring each other.
    if red_threatens and red_distance < cfg.very_close_distance:
        return escape()

    # 2. Facing us, close but not yet critical: turn right.
    if red_threatens and red_distance < cfg.red_turn_distance:
        return cfg.forward_speed, right, 'AVOID_RED'

    # 3. Any green back in view: steer onto it, easing off so we settle near
    #    cruise_distance rather than driving into it. There is no separate
    #    "approach a green" case - the speed ramp already runs at full speed
    #    when it is far away, so following and approaching a green are the
    #    same behaviour. Collapsing them removes the FOLLOW <-> APPROACH flip
    #    that let a robot abandon its leader and swing at something else.
    if green:
        follow_cap = min(cfg.max_angular_speed,
                         cfg.forward_speed / max(cfg.min_follow_radius, 1e-6))
        turn = clamp(cfg.steering_gain * green_bearing,
                     -follow_cap, follow_cap)
        reach = max(cfg.cruise_distance - cfg.close_distance, 1e-6)
        ratio = clamp((green_distance - cfg.close_distance) / reach, 0.0, 1.0)
        speed = cfg.min_speed + (cfg.forward_speed - cfg.min_speed) * ratio
        return speed, turn, 'FOLLOW'

    # 4. No green anywhere, but a robot is out there facing us and far
    #    enough not to be a threat: head towards it to join up.
    if red and red_distance >= cfg.approach_distance:
        approach_cap = min(cfg.max_angular_speed,
                           cfg.forward_speed / max(cfg.approach_min_radius, 1e-6))
        turn = clamp(cfg.steering_gain * red_bearing,
                     -approach_cap, approach_cap)
        return cfg.forward_speed, turn, 'APPROACH'

    # 5. No robot identified by the camera, but the LiDAR sees something
    #    inside escape_clear_range. It may be a wall, or a peer the camera
    #    has lost - at contact range it routinely does, and a peer's wheel
    #    alone reads as an anonymous obstacle. We no longer try to tell them
    #    apart: the response is the same escape as rule 1, so whichever it
    #    is, the robot backs out if touching and otherwise rotates until the
    #    way ahead is clear, then drives off in that direction.
    if not green and not red and ahead < cfg.escape_clear_range:
        return escape('OBSTACLE')

    # Nothing to act on: wander on the base curve.
    return cfg.forward_speed, clamp(cfg.base_angular_speed,
                                    -cfg.max_angular_speed,
                                    cfg.max_angular_speed), 'BASE_CURVE'
