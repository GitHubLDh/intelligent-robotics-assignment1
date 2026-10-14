"""Stateless bearing-and-distance reactive control for differential-drive robots.

Control Law:
1. Circle Trajectory (BASE_CURVE, no target visible):
   v = forward_speed, w = base_angular_speed
   Since linear velocity v and angular velocity w are constant non-zero,
   the robot moves along a circle of radius R = v / |w|.

2. Target Following (FOLLOW, target visible):
   - Steering: Proportional steering toward the detected target bearing drives
     bearing -> 0 (keeping the target straight ahead).
     w = clamp(steering_gain * bearing_rad, -max_angular_speed, +max_angular_speed)
   - Distance keeping: Proportional control maintaining 2x robot size (0.50 m).
     Target distance = robot_size * distance_multiplier = 0.25 * 2.0 = 0.50 m.
     When distance_m is available:
       v_cmd = forward_speed + distance_gain * (distance_m - target_distance)
       v = clamp(v_cmd, 0.0, max_forward_speed)
     If too close (e.g. <= 0.30 m), the robot stops (v = 0) to avoid collisions.
     If farther than 0.50 m, it speeds up to catch up.
     At 0.50 m, it cruises at forward_speed to match the leader.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlParameters:
    forward_speed: float = 0.16          # m/s; nominal forward speed
    base_angular_speed: float = -0.20    # rad/s; default circular arc
    steering_gain: float = 1.80          # (rad/s)/rad
    max_angular_speed: float = 0.90      # rad/s
    robot_size: float = 0.25             # m; diameter of the robot
    distance_multiplier: float = 2.0     # keep 2x robot size
    target_distance: float = 0.50        # m; 2 * robot_size = 0.50 m
    distance_gain: float = 0.80          # (m/s)/m; P-gain for distance control
    max_forward_speed: float = 0.22      # m/s; maximum forward catch-up speed


def command_for_detection(visible, bearing_rad, cfg, distance_m=None):
    """Return (linear_m_s, angular_rad_s, mode) from *current* detection.

    - When visible:
        * P-steering toward bearing_rad drives bearing -> 0 (straight ahead).
        * Distance control adjusts speed to maintain 2x robot size (target_distance).
    - When not visible:
        * Moves along circle according to individual base curvature.
    """
    if visible and bearing_rad is not None and math.isfinite(bearing_rad):
        # 1. Steering control to keep target straight ahead
        w = max(-cfg.max_angular_speed,
                min(cfg.max_angular_speed, cfg.steering_gain * bearing_rad))

        # 2. Distance control to maintain 2x the robot size
        if distance_m is not None and math.isfinite(distance_m):
            target_d = cfg.target_distance
            v_cmd = cfg.forward_speed + cfg.distance_gain * (distance_m - target_d)
            v = max(0.0, min(cfg.max_forward_speed, v_cmd))
        else:
            v = cfg.forward_speed

        return v, w, "FOLLOW"

    w = max(-cfg.max_angular_speed,
            min(cfg.max_angular_speed, cfg.base_angular_speed))
    return cfg.forward_speed, w, "BASE_CURVE"
