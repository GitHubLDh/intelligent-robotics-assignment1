"""Camera-only, per-frame front/rear target classification.

Recognition model names denote *types* of objects, not robot identity.
Marker visibility alone is not directional in Webots. We associate markers
with observed `simple_diff_robot` body centers by their camera-relative
positions and accept only the marker on the camera-facing side of that body.

No getId/getOrientation, Supervisor, global coordinates or target history.
No body association => reject candidate (conservative fail-safe).
"""
import math

MAX_RANGE_M = 4.0
HALF_FOV_RAD = math.radians(50)
MAX_MARKER_TO_BODY_M = 0.22
MIN_ASSOCIATION_MARGIN_M = 0.045
MIN_NEAR_SIDE_PROJECTION_M = 0.035
MIN_FRONT_REAR_PROJECTION_GAP_M = 0.050
# Where a follower's body centre sits when correctly in formation, measured
# out from its leader's rear marker: desired_rear_range (0.275) is a
# camera-to-marker range and the camera is 0.130 ahead of its own centre.
FOLLOW_SLOT_STANDOFF_M = 0.40
# Tolerance on that point: hold_tolerance plus cornering slack.
SLOT_RADIUS_M = 0.26
# A leader turning broadside fails the directional marker test (neither marker
# projects clearly nearer) while its BODY is still detected. Within this range
# we follow the body so a train survives the turn. Deliberately short: far
# bodies are not plausibly the robot we were queueing behind.
BODY_FALLBACK_RANGE_M = 2.0
# Marker-based following is ANTISYMMETRIC: seeing a robot's green back means
# it faces away, so it cannot see us, and mutual following is impossible. A
# body carries no direction, so body-following is symmetric and five robots
# became a mutual attraction sink that settled into everyone FOLLOW_HOLDing
# at desired_rear_range from everyone else. This floor restores the asymmetry:
#   piled up (centres ~0.4m apart)  -> camera reads ~0.27m -> refused
#   real leader at desired_rear_range -> 0.275 + 0.126     -> 0.401m, accepted
# It also sits above FOLLOW_HOLD's desired_rear_range + hold_tolerance (0.335),
# so a body target can never trigger a hold.
BODY_FALLBACK_MIN_M = 0.36


def _measurement(obj):
    try:
        x, y, z = [float(q) for q in obj.getPosition()]
    except (TypeError, ValueError, AttributeError):
        return None
    if not all(math.isfinite(v) for v in (x, y, z)):
        return None
    distance = math.hypot(x, y)
    bearing = math.atan2(y, x)
    if x <= 0.0 or not (0.16 < distance <= MAX_RANGE_M) or abs(bearing) > HALF_FOV_RAD:
        return None
    return (x, y, z, distance, bearing)


def _pair_to_bodies(markers, bodies):
    """Associate each observed marker with one unambiguous nearby body.

    Uses only position measurements from the *same frame*. Uncertain
    associations are ignored; we never pair markers by global IDs or history.
    """
    grouped = [{'front': [], 'rear': []} for _ in bodies]
    orphan = 0
    for kind, m in markers:
        choices = sorted((math.dist(m[:3], b[:3]), i) for i, b in enumerate(bodies))
        if not choices or choices[0][0] > MAX_MARKER_TO_BODY_M:
            orphan += 1
            continue
        if len(choices) > 1 and choices[1][0] - choices[0][0] < MIN_ASSOCIATION_MARGIN_M:
            orphan += 1
            continue
        grouped[choices[0][1]][kind].append(m)
    return grouped, orphan


def _slot_centre(body, marker):
    """The point a follower's body occupies behind this green back.

    The marker-minus-body vector points backwards along that robot's own
    heading, so it gives the slot direction without any global pose.
    """
    dx, dy = marker[0] - body[0], marker[1] - body[1]
    span = math.hypot(dx, dy)
    if span < 1e-6:
        return None
    return (marker[0] + dx / span * FOLLOW_SLOT_STANDOFF_M,
            marker[1] + dy / span * FOLLOW_SLOT_STANDOFF_M)


def _occupant(slot, bodies, leader_index):
    """Index of a body already parked in that slot, else None."""
    for i, other in enumerate(bodies):
        if i == leader_index:
            continue
        if math.dist(other[:2], slot) <= SLOT_RADIUS_M:
            return i
    return None


def _lane_offset(body, marker):
    """Signed distance from our camera to the leader's rear axis.

    Positive means we are to the right of the lane. The follow controller
    steers only on marker *bearing*, which is satisfied from any direction,
    so two robots converging on one leader from different angles both read
    bearing ~0 and drive along converging lines into each other. This term
    tells each of them how far off the leader's lane it is, so they queue up
    single file instead. Zero when already centred, so it adds nothing to a
    correctly formed train.
    """
    ux, uy = marker[0] - body[0], marker[1] - body[1]
    span = math.hypot(ux, uy)
    if span < 1e-6:
        return 0.0
    ux, uy = ux / span, uy / span
    return uy * marker[0] - ux * marker[1]


def _body_fallback(bodies, red_bodies):
    """Follow a detected body when no green back can be classified.

    Measured cause of trains breaking up: once the angle between a leader's
    heading and our line of sight to it passes ~74 degrees, neither of its
    markers projects clearly nearer, so the directional test fails closed and
    reports nothing - even though the body is still detected in that same
    frame. Following the body keeps the train through the turn.

    Uses only the CURRENT frame: no remembered target, bearing or tracking ID.
    Bodies carrying a classified RED front marker are skipped - those are
    pointed at us, and following one would mean driving at an oncoming robot.
    """
    # Anything this close means we are already in a cluster. Latching onto it
    # is what produced the five-robot gridlock, so refuse outright and let the
    # base behaviour carry us out instead of adding to the pile.
    if any(b[3] < BODY_FALLBACK_MIN_M for b in bodies):
        return None
    usable = [b for i, b in enumerate(bodies)
              if i not in red_bodies and b[3] <= BODY_FALLBACK_RANGE_M]
    if not usable:
        return None
    nearest = min(usable, key=lambda b: b[3])
    # Heading is unknown without a classified marker, so the lane term gets
    # 0.0 rather than a guessed direction.
    return (nearest[3], nearest[4], 0.0)


def choose_follow_target(green, bodies, red_bodies=frozenset()):
    """Pick the green back whose follow slot is still free.

    Following the *nearest* green back made two robots converge on one point
    behind the same leader and collide sideways. A green back with another
    robot body already standing in its slot is taken, so we skip it and aim
    for the tail of that train instead.

    The occupant is found from body detections, not from its markers: a robot
    seen broadside has neither marker closer to us, so it is unclassifiable,
    and a follower's red marker points at its own leader and is invisible from
    outside the train. Its body, however, is always detected.

    Falls back to the nearest green back when every slot looks taken, so a
    robot never ends up with no target and wanders away from the group.

    With no classifiable green back at all, falls back to a detected body -
    see _body_fallback.
    """
    if not green:
        return _body_fallback(bodies, red_bodies)
    ordered = sorted(green, key=lambda c: c[0])
    for distance, bearing, index, body, marker in ordered:
        slot = _slot_centre(body, marker)
        if slot is None:
            continue
        taken_by = _occupant(slot, bodies, index)
        if taken_by is None:
            return (distance, bearing, _lane_offset(body, marker))
        if taken_by not in {c[2] for c in ordered}:
            # The tail's own green back is not classifiable from here, so aim
            # at its body centre. That is ~0.13m short of where its marker
            # would be, which desired_rear_range's tolerance absorbs.
            # No marker for the tail, so its lane is unknown: 0.0 disables
            # the lane term rather than guessing a direction.
            return (bodies[taken_by][3], bodies[taken_by][4], 0.0)
        # The tail IS a green candidate; it is evaluated on its own turn.
    return (ordered[0][0], ordered[0][1],
            _lane_offset(ordered[0][3], ordered[0][4]))


def scan_markers(recognition_objects):
    """Return accepted rear/front (distance,bearing), counts and diagnostics.

    Body-centered radial projection rejects the *far* marker of the same
    robot. In a side-on view neither marker projects clearly closer, so
    neither is classified as a safe following target.
    """
    model_types = set()
    raw_count = 0
    bodies = []
    markers = []
    for obj in recognition_objects:
        raw_count += 1
        model = str(obj.getModel())
        model_types.add(model)
        if model not in ('simple_diff_robot', 'rear_marker', 'front_marker'):
            continue
        m = _measurement(obj)
        if m is None:
            continue
        if model == 'simple_diff_robot':
            bodies.append(m)
        else:
            markers.append(('rear' if model == 'rear_marker' else 'front', m))

    groups, orphan_count = _pair_to_bodies(markers, bodies)
    green = []
    red = []
    red_bodies = set()
    uncertain = 0
    for body_index, (body, group) in enumerate(zip(bodies, groups)):
        # The body direction is a ray from THIS camera, not a global heading.
        ux, uy = body[0] / body[3], body[1] / body[3]
        # Marker distance toward camera measured along body bearing.
        near_front = [((body[0]-m[0])*ux+(body[1]-m[1])*uy, m)
                      for m in group['front']]
        near_rear = [((body[0]-m[0])*ux+(body[1]-m[1])*uy, m)
                     for m in group['rear']]
        # Multiple candidate marker detections for one body: fail closed.
        if len(near_front)>1 or len(near_rear)>1:
            uncertain += 1
            continue
        f = near_front[0] if near_front else None
        r = near_rear[0] if near_rear else None
        if f and r:
            if (f[0] >= MIN_NEAR_SIDE_PROJECTION_M and
                    f[0] - r[0] >= MIN_FRONT_REAR_PROJECTION_GAP_M):
                red.append(f[1][3:5])
                red_bodies.add(body_index)
            elif (r[0] >= MIN_NEAR_SIDE_PROJECTION_M and
                    r[0] - f[0] >= MIN_FRONT_REAR_PROJECTION_GAP_M):
                green.append(r[1][3:5] + (body_index, body, r[1]))
            else:
                uncertain += 1
        elif f:
            if f[0] >= MIN_NEAR_SIDE_PROJECTION_M:
                red.append(f[1][3:5])
                red_bodies.add(body_index)
            else:
                uncertain += 1
        elif r:
            if r[0] >= MIN_NEAR_SIDE_PROJECTION_M:
                green.append(r[1][3:5] + (body_index, body, r[1]))
            else:
                uncertain += 1

    target = choose_follow_target(green, bodies, red_bodies)
    # Slide 4 names the target as the NEAREST VISIBLE ROBOT, so the reactive
    # controller uses this directly. The marker-derived fields below are kept
    # for diagnostics and for the oncoming-robot extension only.
    nearest_robot = min(bodies, key=lambda b: b[3])[3:5] if bodies else None
    return {
        'nearest_robot': nearest_robot,
        'rear': target[:2] if target else None,
        'lane_offset': target[2] if target else 0.0,
        'front': min(red) if red else None,
        'count': len(green) + len(red),
        'raw_count': raw_count,
        'model_types': sorted(model_types),
        'rear_count': len(green),
        'front_count': len(red),
        'uncertain_count': uncertain,
        'unmatched_count': orphan_count,
        'body_count': len(bodies),
    }
