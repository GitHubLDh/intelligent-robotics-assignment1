"""Synchronized camera + full LiDAR snapshot; no target history/global pose.

Float64MultiArray: [green distance, green bearing, red distance, red bearing,
                    360 laser distances in meters].
The scan's index 180 is robot +X (forward), index 90 is right.
"""
from math import nan, isfinite

N_BEAMS = 360
CAMERA_FIELDS = ('distance_m', 'bearing_rad', 'front_distance_m', 'front_bearing_rad')


def sensor_packet(data):
    scan = data.get('lidar_ranges_m')
    if not isinstance(scan, (list, tuple)) or len(scan) != N_BEAMS:
        scan = [nan] * N_BEAMS
    elif any(not isinstance(r, (int, float)) or not isfinite(r) or r < 0 or r > 2.51
             for r in scan):
        scan = [nan] * N_BEAMS
    result = []
    for i, key in enumerate(CAMERA_FIELDS):
        detected = bool(data.get('visible' if i < 2 else 'front_visible', False))
        value = data.get(key) if detected else None
        try:
            result.append(float(value) if value is not None else nan)
        except (TypeError, ValueError):
            result.append(nan)
    return result + list(scan)
