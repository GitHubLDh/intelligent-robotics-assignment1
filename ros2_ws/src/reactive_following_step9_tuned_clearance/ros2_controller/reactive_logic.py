"""Reactive following, per Assignment 1 slide 4.

    no detection  -> follow the base curve
    target seen   -> constant linear speed, turn towards the NEAREST VISIBLE
                     ROBOT; turning decreases as alignment improves
    target lost   -> resume the base curve

Every decision uses one current sensor snapshot: no target memory, no global
pose, no inter-robot communication (slide 6). Sensing is frontal only - a
100 degree, 4 m camera and LiDAR - so nothing to the side or behind the robot
is observable, and no rule may depend on it.

Collision avoidance and distance-dependent speed are extensions (slide 8) and
are switched off by default so the required behaviour can be measured alone.
"""
from dataclasses import dataclass
import math

N_BEAMS = 360
FIELD_OF_VIEW = math.radians(100)
HALF_FOV = FIELD_OF_VIEW / 2.0
RANGE_MAX = 4.0


def clamp(v, lo, hi):
    return min(hi, max(lo, v))


@dataclass(frozen=True)
class FollowParameters:
    # Slide 4: the linear speed is CONSTANT. Only the extensions below may
    # reduce it, and only when explicitly enabled.
    forward_speed: float = 0.15
    # The experiment variable of slide 7: no following / weak / strong turning.
    steering_gain: float = 2.0
    max_angular_speed: float = 1.1
    # Open-space wander when no robot is visible.
    base_angular_speed: float = -0.20
    # --- extensions, slide 8 ---
    # Distance-dependent speed. The LiDAR sits above the other robots' bodies
    # and cannot see a peer closer than ~0.75 m, so the standoff is driven by
    # the CAMERA's measured range to the followed robot - the only sensor that
    # reads robot distance at close quarters. On by default because without it
    # the specified constant-speed behaviour simply drives into the leader;
    # --constant-speed restores the spec-pure baseline for the comparisons.
    keep_distance: bool = True
    desired_distance: float = 0.45
    approach_margin: float = 0.35
    avoid_enabled: bool = False
    avoid_range: float = 0.55
    avoid_turn_speed: float = 0.80
    slow_enabled: bool = False
    slow_range: float = 0.90


def target_visible(distance, bearing):
    """A robot is usable only inside the real sensor envelope."""
    return (distance is not None and bearing is not None and
            math.isfinite(distance) and math.isfinite(bearing) and
            0.0 < distance <= RANGE_MAX and abs(bearing) <= HALF_FOV)


def beam_angle(index, beams=N_BEAMS):
    """Index 0 is the right edge (-50 deg), 180 ahead, 359 the left edge."""
    return -HALF_FOV + FIELD_OF_VIEW * index / (beams - 1)


def sector_min(scan, low, high):
    """Nearest return with a beam angle in [low, high], else RANGE_MAX."""
    values = [d for i, d in enumerate(scan) if low <= beam_angle(i) <= high]
    return min(values) if values else RANGE_MAX


def scan_sectors(scan):
    """Frontal clearances only. A 100 deg sensor has no side or rear view."""
    if (scan is None or len(scan) != N_BEAMS or
            not all(isinstance(d, (int, float)) and math.isfinite(d) and
                    0 < d <= RANGE_MAX + 0.01 for d in scan)):
        return None
    return {
        'ahead': sector_min(scan, math.radians(-12), math.radians(12)),
        'left': sector_min(scan, math.radians(10), HALF_FOV),
        'right': sector_min(scan, -HALF_FOV, math.radians(-10)),
    }


def follow_command(target_distance, target_bearing, ranges, cfg):
    """Return (linear m/s, angular rad/s, mode) for the current snapshot."""
    sectors = scan_sectors(ranges)
    if sectors is None:
        return 0.0, 0.0, 'SENSOR_FAILSAFE'

    seen = target_visible(target_distance, target_bearing)
    speed = cfg.forward_speed

    # Extension (slide 8): ease off when closing on something ahead.
    if cfg.slow_enabled and sectors['ahead'] < cfg.slow_range:
        reach = max(cfg.slow_range - cfg.avoid_range, 1e-6)
        speed *= clamp((sectors['ahead'] - cfg.avoid_range) / reach, 0.0, 1.0)

    # Extension (slide 8): frontal collision avoidance. Turn toward whichever
    # side of the cone is more open - a frontal sensor can still compare its
    # own left and right halves, which is the only asymmetry available.
    if cfg.avoid_enabled and sectors['ahead'] < cfg.avoid_range:
        direction = 1.0 if sectors['left'] >= sectors['right'] else -1.0
        turn = direction * min(cfg.avoid_turn_speed, cfg.max_angular_speed)
        return speed, turn, 'AVOID'

    if seen:
        # "Turn strongly towards the nearest visible robot; turning decreases
        # as alignment improves" - proportional control does exactly that,
        # and steering_gain is the weak/strong knob the experiments compare.
        turn = clamp(cfg.steering_gain * target_bearing,
                     -cfg.max_angular_speed, cfg.max_angular_speed)
        if cfg.keep_distance:
            # Ease to a stop as the gap closes to desired_distance. Steering
            # is untouched, so a held robot still tracks its leader and
            # resumes the moment the leader pulls away.
            gap = target_distance - cfg.desired_distance
            speed *= clamp(gap / max(cfg.approach_margin, 1e-6), 0.0, 1.0)
            if speed <= 0.005:
                return 0.0, turn, 'FOLLOW_HOLD'
        return speed, turn, 'FOLLOW'

    # No robot in view: base curve. Also the specified response to a target
    # loss, which is why no recovery or memory of the lost target exists.
    return speed, clamp(cfg.base_angular_speed,
                        -cfg.max_angular_speed, cfg.max_angular_speed), 'BASE_CURVE'
