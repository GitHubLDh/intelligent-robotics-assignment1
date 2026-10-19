"""Camera recognition of coloured robot halves.

Each robot's front half is red and its rear half green, and each half is its
own recognised Solid. So telling "a robot facing me" from "a robot facing
away" is just a question of which colour is nearer - no marker-to-body
association, no projection geometry, no fail-closed cases.

Reports only the nearest red and the nearest green as (distance, bearing)
from the camera. Current frame only: no identities, history or global pose.
"""
import math

MAX_RANGE_M = 4.0
HALF_FOV_RAD = math.radians(50)
MIN_RANGE_M = 0.05


def _measurement(obj):
    """Camera-relative (distance, bearing), or None if outside the envelope."""
    try:
        x, y, _ = [float(q) for q in obj.getPosition()]
    except (TypeError, ValueError, AttributeError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    distance = math.hypot(x, y)
    bearing = math.atan2(y, x)
    if x <= 0.0 or not (MIN_RANGE_M < distance <= MAX_RANGE_M):
        return None
    if abs(bearing) > HALF_FOV_RAD:
        return None
    return (distance, bearing)


def scan_markers(recognition_objects):
    """Nearest red half and nearest green half currently in view."""
    red, green = [], []
    models = set()
    raw = 0
    for obj in recognition_objects:
        raw += 1
        model = str(obj.getModel())
        models.add(model)
        if model not in ('front_half', 'rear_half'):
            continue
        measured = _measurement(obj)
        if measured is None:
            continue
        (red if model == 'front_half' else green).append(measured)
    return {
        'front': min(red) if red else None,
        'rear': min(green) if green else None,
        'front_count': len(red),
        'rear_count': len(green),
        'count': len(red) + len(green),
        'raw_count': raw,
        'model_types': sorted(models),
    }
