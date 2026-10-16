"""Synchronized camera + full LiDAR snapshot; no target history/global pose.

Float64MultiArray: [green distance, green bearing, red distance, red bearing,
                    lane offset, 360 laser distances in meters].
The scan's index 180 is robot +X (forward), index 90 is right.
"""
from math import nan, isfinite

N_BEAMS = 360
# Must track the LiDAR's maxRange in SimpleDiffRobot.proto and lidar_scan.py.
RANGE_MAX_M = 4.0
CAMERA_FIELDS = ('distance_m', 'bearing_rad', 'front_distance_m', 'front_bearing_rad')


def sensor_packet(data):
    scan = data.get('lidar_ranges_m')
    if not isinstance(scan, (list, tuple)) or len(scan) != N_BEAMS:
        scan = [nan] * N_BEAMS
    elif any(not isinstance(r, (int, float)) or not isfinite(r) or r < 0
             or r > RANGE_MAX_M + 0.01 for r in scan):
        scan = [nan] * N_BEAMS
    result = []
    for i, key in enumerate(CAMERA_FIELDS):
        detected = bool(data.get('visible' if i < 2 else 'front_visible', False))
        value = data.get(key) if detected else None
        try:
            result.append(float(value) if value is not None else nan)
        except (TypeError, ValueError):
            result.append(nan)
    lane = data.get('lane_offset_m')
    try:
        result.append(float(lane) if lane is not None else 0.0)
    except (TypeError, ValueError):
        result.append(0.0)
    return result + list(scan)
