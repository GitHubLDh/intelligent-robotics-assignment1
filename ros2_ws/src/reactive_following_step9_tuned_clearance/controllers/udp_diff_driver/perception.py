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
    uncertain = 0
    for body, group in zip(bodies, groups):
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
            elif (r[0] >= MIN_NEAR_SIDE_PROJECTION_M and
                    r[0] - f[0] >= MIN_FRONT_REAR_PROJECTION_GAP_M):
                green.append(r[1][3:5])
            else:
                uncertain += 1
        elif f:
            if f[0] >= MIN_NEAR_SIDE_PROJECTION_M:
                red.append(f[1][3:5])
            else:
                uncertain += 1
        elif r:
            if r[0] >= MIN_NEAR_SIDE_PROJECTION_M:
                green.append(r[1][3:5])
            else:
                uncertain += 1

    return {
        'rear': min(green) if green else None,
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
