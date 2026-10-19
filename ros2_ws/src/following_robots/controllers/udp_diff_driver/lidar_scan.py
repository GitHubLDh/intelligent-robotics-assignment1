"""Webots 3-layer FRONTAL LiDAR -> instantaneous 360-sample local scan.

100 deg horizontal field of view sampled by 360 beams, so index 0 is the
right-hand edge (-50 deg), index 180 is straight ahead, and index 359 is the
left-hand edge (+50 deg). Webots range-image layers are contiguous. No poses,
target identities, or saved observations.
"""
import math

N_BEAMS = 360
MAX_M = 4.0
MIN_M = 0.05


def combine_layers(raw, beams=N_BEAMS, layers=3, max_range=MAX_M):
    """Min valid depth across layers; +inf means unobstructed maximum range.

    Return None if the scan isn't ready or has no valid beams. NaNs and
    nonpositive depths are NOT silently interpreted as unobstructed space.
    """
    if raw is None or len(raw) != beams * layers:
        return None
    scan = []
    valid_count = 0
    for i in range(beams):
        readings = []
        for layer in range(layers):
            v = float(raw[layer * beams + i])
            if math.isinf(v) and v > 0:
                readings.append(max_range)
            elif math.isfinite(v):
                # Values below minRange are collision risks, not free space.
                readings.append(max(MIN_M, min(max_range, v)))
        if readings:
            valid_count += 1
            scan.append(round(min(readings), 3))
        else:
            scan.append(0.0)  # unsafe/unknown beam
    return scan if valid_count >= beams * 0.8 else None
