"""Local relative robot measurements from Webots front_camera recognition.

No Supervisor API, world coordinates, target identifiers or target memory.
This module uses only the camera's currently visible recognition objects.
"""

import math


MAX_RANGE_M = 4.0
HALF_FOV_RAD = math.radians(50.0)
ROBOT_MODEL = "simple_diff_robot"


def select_nearest_robot(recognition_objects):
    """Return (distance in m, left-positive bearing in rad, valid count) or None.

    Webots camera coordinates: +X forward, +Y left, +Z up.
    The model field identifies robot objects, not their individual identity.
    """
    valid = []
    for item in recognition_objects:
        if item.getModel() != ROBOT_MODEL:
            continue
        x, y, _z = item.getPosition()
        distance = math.hypot(x, y)
        bearing = math.atan2(y, x)
        # Ignore targets behind camera and any own-body artifacts.
        if x <= 0.0 or distance < 0.16 or distance > MAX_RANGE_M:
            continue
        if abs(bearing) > HALF_FOV_RAD:
            continue
        valid.append((distance, bearing))

    if not valid:
        return None
    nearest = min(valid, key=lambda entry: entry[0])
    return nearest[0], nearest[1], len(valid)
