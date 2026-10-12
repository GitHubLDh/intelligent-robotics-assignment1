"""Stateless bearing-only reactive control for a differential-drive robot.

There is no target identity, remembered target location, or prediction.
Sensor freshness/visibility checks live in follow_r1.py.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlParameters:
    forward_speed: float = 0.16       # m/s; constant in both states
    base_angular_speed: float = -0.20 # rad/s; default clockwise arc
    steering_gain: float = 1.80       # (rad/s)/rad
    max_angular_speed: float = 0.90  # rad/s


def command_for_detection(visible, bearing_rad, cfg):
    """Return (linear_m_s, angular_rad_s, mode) from *current* detection.

    A detected robot gives P-steering toward its left-positive bearing.
    Without a valid detection, return to the individual base curvature.
    """
    if (visible and bearing_rad is not None
            and math.isfinite(bearing_rad)):
        w = max(-cfg.max_angular_speed,
                min(cfg.max_angular_speed, cfg.steering_gain * bearing_rad))
        return cfg.forward_speed, w, "FOLLOW"

    w = max(-cfg.max_angular_speed,
            min(cfg.max_angular_speed, cfg.base_angular_speed))
    return cfg.forward_speed, w, "BASE_CURVE"
