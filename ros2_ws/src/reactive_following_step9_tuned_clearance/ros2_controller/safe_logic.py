"""Stateless local camera + 360-degree LiDAR reactive control.

No remembered targets, tracking IDs, GPS/global positions, Supervisor API, or
inter-robot communication. Every decision uses ONE current sensor snapshot.
LiDAR distances are from the LiDAR origin, so margins include robot footprint.
"""
from dataclasses import dataclass
import math

N_BEAMS = 360
RANGE_MAX = 2.5
COLLISION_BACKOFF_S = 0.50
COLLISION_BACKOFF_SPEED = -0.10


def clamp(v, lo, hi):
    return min(hi, max(lo, v))


def collision_recovery_command(now, backoff_until, restart_at):
    """Return a collision recovery command, or None when recovery is done."""
    if now < backoff_until:
        return COLLISION_BACKOFF_SPEED, 0.0, 'COLLISION_BACKOFF'
    if now < restart_at:
        return 0.0, 0.0, 'COLLISION_WAIT'
    return None


@dataclass(frozen=True)
class SafeParameters:
    forward_speed: float = 0.16
    follow_max_speed: float = 0.21
    base_angular_speed: float = -0.20
    steering_gain: float = 2.6
    max_angular_speed: float = 1.1
    desired_rear_range: float = 0.275
    hold_tolerance: float = 0.06
    distance_gain: float = 0.65
    tracking_max_turn_speed: float = 0.85
    tracking_edge_bearing: float = 0.82
    head_on_range: float = 0.70
    head_on_turn_speed: float = 0.95
    head_on_reverse_turn_speed: float = 0.60
    avoid_angular_speed: float = 0.75
    lidar_slow_range: float = 0.73
    lidar_front_avoid: float = 0.50
    lidar_emergency: float = 0.28
    lidar_side_avoid: float = 0.28
    lidar_rear_reverse_clear: float = 0.55
    # Range at which an oncoming robot starts biasing us right. Deliberately
    # much larger than head_on_range so the paths diverge before a head-on
    # can develop, and so backing off cannot drop us out of the behaviour.
    oncoming_range: float = 1.80
    # Kept gentle: a robot that swings more than ~50 degrees drops out of its
    # own follower's camera cone, breaking a train that had already formed.
    oncoming_bias_speed: float = 0.42


def marker_visible(distance, bearing):
    return (distance is not None and bearing is not None and
            math.isfinite(distance) and math.isfinite(bearing) and
            0.16 < distance <= 4.0 and abs(bearing) <= math.radians(50))


def beam_angle(i, beams=N_BEAMS):
    """Positive angle to the left. Index N/2 is +X/forward."""
    return -math.pi + 2.0 * math.pi * i / beams


def angular_distance(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def lidar_sectors(scan, rear_distance=None, rear_bearing=None):
    """Instantaneous safe clearances in eight sectors.

    A LiDAR point can represent the currently visible green-marked robot, so
    exclude only readings compatible with the current *bearing and distance*.
    Other returns (especially closer walls) remain active collision hazards.
    Raw emergency ranges are also returned; never mask a too-close collision.
    """
    if len(scan) != N_BEAMS or not all(
            isinstance(d, (int, float)) and math.isfinite(d) and 0 < d <= RANGE_MAX+0.01
            for d in scan):
        return None

    green = marker_visible(rear_distance, rear_bearing)
    compensated = []
    for i, d in enumerate(scan):
        a = beam_angle(i)
        # Marker distance is camera-relative, from x=+0.13m. LiDAR origin is
        # near robot center; allow for robot body and marker offset.
        half_width = clamp(0.17 / max(rear_distance or 1.0, .3), .16, .38)
        # Our camera sits at x=+0.130 and the leader's green mark at its own
        # x=-0.126, while our LiDAR is at body centre: the leader's return is
        # therefore about 0.13m FARTHER than the marker reading, never nearer.
        # The old -0.32 lower bound erased anything up to 32cm closer than the
        # marker - including a rival robot converging on the same slot, which
        # left it invisible to the side and diagonal avoidance below.
        is_leader = (green and angular_distance(a, rear_bearing) < half_width and
                     rear_distance - .08 <= d <= rear_distance + .38 and d > .20)
        compensated.append(RANGE_MAX if is_leader else d)

    def nearest(values, angle, halfwidth):
        return min(d for i, d in enumerate(values)
                   if angular_distance(beam_angle(i), angle) <= halfwidth)

    front = nearest(compensated, 0, math.radians(18))
    front_left = nearest(compensated, math.radians(40), math.radians(20))
    front_right = nearest(compensated, math.radians(-40), math.radians(20))
    left = nearest(compensated, math.pi/2, math.radians(27))
    right = nearest(compensated, -math.pi/2, math.radians(27))
    rear_left = nearest(compensated, math.radians(140), math.radians(24))
    rear_right = nearest(compensated, math.radians(-140), math.radians(24))
    rear = nearest(compensated, math.pi, math.radians(26))
    raw_front = nearest(scan, 0, math.radians(18))
    return {
        'front':front,'front_left':front_left,'front_right':front_right,
        'left':left,'right':right,'rear_left':rear_left,
        'rear_right':rear_right,'rear':rear,'raw_front':raw_front,
    }


def safe_command(rear_distance, rear_bearing, front_distance, front_bearing,
                 ranges, cfg: SafeParameters):
    """Return (linear m/s, angular rad/s, mode) from current local sensors."""
    s = lidar_sectors(ranges, rear_distance, rear_bearing) if ranges is not None else None
    if s is None:
        return 0.0, 0.0, 'SENSOR_FAILSAFE'

    f, fl, fr = s['front'], s['front_left'], s['front_right']
    l, r, b = s['left'], s['right'], s['rear']
    bl, br = s['rear_left'], s['rear_right']
    green = marker_visible(rear_distance, rear_bearing)
    red = marker_visible(front_distance, front_bearing)
    # Contested follow slots are resolved in perception.py, which can see the
    # body positions this controller never receives: it reports the tail of a
    # train rather than its nearest member, so rear_distance/rear_bearing are
    # already a slot nobody else owns.
    red_ahead = (red and abs(front_bearing) < .85
                 and front_distance <= cfg.head_on_range
                 and (not green or front_distance <= rear_distance + .18))

    # Deterministic clockwise-only avoidance is deliberately conservative.
    # Previous versions flipped from -0.8 to +0.8 as the right-side/diagonal
    # distances crossed a threshold, causing oscillation in R2's corner.
    # Side collision checks below STOP / BACK OFF if a pivot is unsafe: they
    # never issue a contradictory anticlockwise turn on a subsequent scan.
    # This is one-frame reactive control: NO remembered direction/target.
    escape_w = -min(cfg.avoid_angular_speed, cfg.max_angular_speed)
    headon_w = -min(cfg.head_on_turn_speed, cfg.max_angular_speed)
    headon_direction = -1.0
    reverse_clear = b > cfg.lidar_rear_reverse_clear and bl > .33 and br > .33

    # Corner-specific rule: a side-clearance comparison changed yaw sign
    # from frame to frame. Use a deterministic clockwise preference, based
    # only on the current LiDAR scan. No remembered escape state or target.
    # Thresholds are measured from LiDAR origin, not robot surface. Never
    # switch yaw sign in the corner simply because left/right ranges cross.
    # The robot has a nearly circular footprint, so a center pivot does not
    # sweep a greater radius than ordinary rotation. Do not pivot when a
    # wall is already inside the physical footprint + small safety margin.
    # Require a genuinely confined near-corner geometry, not a wall that is
    # still almost a metre away. The robot footprint is approximately 0.16m
    # radius; close-range safety checks below retain a separate hard margin.
    pocket = (f < .72 and fl < .72 and fr < .72 and min(l, r) < .78)
    # Consistent direction for a confined front corridor even if readings
    # straddle the CORNER_CLEAR_TURN / AVOID_DIAGONAL boundary. No history.
    pivot_clear = min(l, r) > .195 and f > .195 and b > .195

    # Critical clearance takes priority over all following and head-on rules.
    # A sufficiently close green-marked leader is a zero-wheel HOLD, not
    # a reason to swivel away. However actual very-close raw returns stop us.
    if s['raw_front'] < .18:
        if reverse_clear:
            return -.040, headon_direction * .35 if red_ahead else 0.0, 'EMERGENCY_REVERSE'
        return 0.0, 0.0, 'EMERGENCY_STOP'

    # In FOLLOW_HOLD, both wheels stop. Keep recognized stationary targets
    # visible, with no remembered bearing or target position.
    if (green and not red_ahead and
        rear_distance <= cfg.desired_rear_range + cfg.hold_tolerance and
        min(l, r) >= .19 and s['raw_front'] >= .18):
        return 0.0, 0.0, 'FOLLOW_HOLD'

    if f < cfg.lidar_emergency:
        # A low wall in front does not automatically require reverse.
        # If a center pivot is safe, rotate toward the clear path first.
        # Otherwise reverse only if all rear LiDAR sectors are clear.
        # Retain the proven right-hand head-on rule for actual red markers.
        if red_ahead and reverse_clear:
            return -.045, headon_direction * min(cfg.head_on_reverse_turn_speed,
                                                   cfg.max_angular_speed), 'YIELD_REVERSE_TURN'
        if pivot_clear:
            return 0.0, headon_w if red_ahead else escape_w, 'BRAKE_TURN'
        if reverse_clear:
            return -.040, 0.0, 'EMERGENCY_BACKOFF'
        return 0.0, 0.0, 'BLOCKED_STOP'

    # Confined corner: commit to a single, locally safe turning preference
    # while the forward gap remains narrow. The former behavior backed up
    # briefly and re-entered BASE_CURVE before the front was actually clear;
    # switching between left/right clearance also caused oscillation.
    #
    # If pivoting is physically unsafe, back up ONLY while rear + rear
    # diagonals are open. When enough room appears, pivot in place toward
    # the deterministic escape side until the CURRENT forward scan clears.
    # No history or previous target/heading is retained.
    corner = f < .56 and ((fl < .43 and l < .37) or
                           (fr < .43 and r < .37) or
                           (fl < .28 and fr < .28))
    if (pocket or corner) and not red_ahead:
        if pivot_clear:
            return 0.0, escape_w, 'CORNER_CLEAR_TURN'
        if reverse_clear:
            return -.048, 0.0, 'CORNER_BACKOFF'
        # Very tight clearance can obstruct both a safe pivot and reverse.
        # Do not change turn direction based on marginal left/right noise;
        # stop rather than command motion into a known occupied sector.
        return 0.0, 0.0, 'CORNER_BLOCKED'

    # Hard side clearance (including behind wheel contact) overrides passing.
    if min(l, r) < .19:
        # A wall is inside the footprint safety margin. Avoid a contact-loaded
        # pivot (or a direction flip) until there is clearance. If the rear is
        # open, back straight out; otherwise stay safely stopped.
        if reverse_clear:
            return -.040, 0.0, 'ESCAPE_SIDE'
        return 0.0, 0.0, 'SIDE_BLOCKED'

    if red_ahead:
        if f < .36 and reverse_clear:
            return -.045, headon_direction * cfg.head_on_reverse_turn_speed, 'YIELD_REVERSE_TURN'
        if f < .48 or (r if headon_direction < 0 else l) < .30:
            return 0.0, headon_w, 'YIELD_TURN'
        # Never advance toward something this close. The old forward creep
        # closed the gap again while turning, feeding the back-off/re-approach
        # cycle. PASS_RIGHT below is what now makes forward progress safely.
        return 0.0, headon_w, 'YIELD'

    # Use front+side coverage to prevent reverse and turning into a wall.
    if f < cfg.lidar_front_avoid:
        # An obstacle straight ahead normally calls for a turn, not a
        # backward-forward cycle. A reverse is reserved for truly confined
        # geometry where spinning cannot clear the robot footprint.
        if pivot_clear:
            return 0.0, escape_w, 'AVOID_FRONT'
        if f < .34 and reverse_clear:
            return -.048, 0.0, 'ESCAPE_REVERSE'
        return 0.0, 0.0, 'BLOCKED_STOP'

    if min(fl, fr) < .34:
        # Same steering preference as BRAKE_TURN and CORNER_CLEAR_TURN.
        # Avoid independently selecting left/right based on one noisy beam.
        return 0.0, escape_w, 'AVOID_DIAGONAL'
    if min(l, r) < cfg.lidar_side_avoid:
        return 0.0, escape_w, 'AVOID_SIDE'

    # Do not allow BASE_CURVE_SLOW to undo a clockwise avoidance turn while
    # the forward corridor is still narrow. In R2's corner this previously
    # caused +base_yaw / -avoid_yaw oscillation around the same position.
    # We only commit to steering on CURRENT obstacles, never a stored escape
    # direction. Front clearance must become comfortably open before moving.
    if not green and not red and f < .88 and min(fl, fr) < .76:
        # Turn early only when an obstacle is genuinely nearby. If the current
        # front, diagonal and side sectors admit a safe curve, retain some
        # forward progress. This removes the old 1.2m stop/very-slow-turn.
        # A centre pivot or backoff is still preferred in cramped geometry.
        if f > .62 and min(fl, fr) > .47 and min(l, r) > .30:
            arc_v = min(.075, max(0.0, .45 * (f - cfg.lidar_front_avoid)))
            return arc_v, escape_w, 'CLEAR_FORWARD_ARC'
        return 0.0, escape_w, 'CLEAR_FORWARD_TURN'

    # If a narrow obstacle remains close in front after its green marker has
    # disappeared, hold momentarily based solely on CURRENT geometry, not
    # target memory. A broad wall also appears in diagonals and is avoided.
    if (not green and not red and .36 < f < cfg.desired_rear_range and
        fl > f + .30 and fr > f + .30 and min(l,r) > .36):
        return 0.0, 0.0, 'PROXIMITY_HOLD'

    if green:
        a = abs(rear_bearing)
        heading_factor = clamp((cfg.tracking_edge_bearing - a) /
                               (cfg.tracking_edge_bearing - .20), 0.0, 1.0)
        if a >= 0.72:
            heading_factor = 0.0  # keep target inside camera before advancing
        error = max(0, rear_distance -
                    (cfg.desired_rear_range + cfg.hold_tolerance))
        v = clamp(cfg.distance_gain * error, 0.0, cfg.follow_max_speed) * heading_factor
        w = clamp(cfg.steering_gain * rear_bearing,
                  -cfg.tracking_max_turn_speed, cfg.tracking_max_turn_speed)
        if a < .025:
            w = 0.0
        # Do not treat the CURRENT visible leader as a wall: sectors above
        # filter only geometrically compatible LiDAR returns. Other obstacles
        # still limit movement before the emergency zone.
        if f < cfg.lidar_slow_range:
            v = min(v, max(0.0, .45 * (f - cfg.lidar_front_avoid)))
        return v, w, 'FOLLOW_ALIGN' if v <= .005 else 'FOLLOW'

    # Base curved motion, with early collision-slowing, even without target.
    # This protects wall approaches that earlier six-sonar sensors missed.
    v = cfg.forward_speed
    if f < cfg.lidar_slow_range:
        v = min(v, max(0.0, .50 * (f - cfg.lidar_front_avoid)))
    # Right-hand passing rule, applied EARLY. The head-on rules above only
    # engage inside head_on_range (0.70m) and the controller then forgot the
    # conflict the instant the gap grew past it: robots backed off, fell
    # straight through to BASE_CURVE, re-aimed and drove into each other
    # again. Biasing from oncoming_range (1.80m) means the two paths have
    # already diverged before a head-on can develop, and backing off cannot
    # drop us out of the behaviour. Both robots turn right, so they separate
    # symmetrically. We keep driving forward rather than pivoting: an arc
    # buys lateral separation with far less heading change than a spin, so a
    # robot already following us keeps our rear marker in its camera cone.
    if red and abs(front_bearing) < 1.0 and front_distance < cfg.oncoming_range:
        urgency = clamp((cfg.oncoming_range - front_distance) /
                        max(cfg.oncoming_range - cfg.head_on_range, .01), .25, 1.0)
        return v, -clamp(cfg.oncoming_bias_speed * urgency,
                         0.0, cfg.max_angular_speed), 'PASS_RIGHT'
    # Preserve clockwise clearance steering near an obstacle, but do NOT
    # bias the base curve from 1.55m away. Current instantaneous scan ONLY.
    near = min(f, fl, fr)
    if near < 1.10:
        weight = clamp((near - .77) / .33, 0.0, 1.0)
        w = (1.0-weight) * escape_w + weight * cfg.base_angular_speed
        return v, w, 'BASE_CLEAR_BLEND'
    return v, clamp(cfg.base_angular_speed, -cfg.max_angular_speed,
                    cfg.max_angular_speed), 'BASE_CURVE_SLOW' if v < cfg.forward_speed else 'BASE_CURVE'
