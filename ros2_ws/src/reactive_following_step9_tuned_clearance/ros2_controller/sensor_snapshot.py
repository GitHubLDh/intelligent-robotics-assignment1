"""Current-camera/current-LiDAR decoding, no target history or poses."""
import math

N_RAYS = 360
SENSOR_VALUES = 5 + N_RAYS


def parse_snapshot(data):
    if len(data) != SENSOR_VALUES:
        return None
    try:
        vals = tuple(float(x) for x in data)
    except (TypeError, ValueError):
        return None
    scan = vals[5:]
    if not all(math.isfinite(x) and 0 < x <= 2.51 for x in scan):
        return None
    camera = tuple(vals[i] if math.isfinite(vals[i]) else None for i in range(4))
    lane = vals[4] if math.isfinite(vals[4]) else 0.0
    return camera + (lane, scan)
