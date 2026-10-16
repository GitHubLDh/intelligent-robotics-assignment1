"""Scatter the five robots to random poses in a world file.

Slide 7 asks for identical initial conditions within a comparison and
"repeat with different positions and headings" across them, so the layout is
seeded: the same --seed always reproduces the same run.

    python3 tools/randomize_starts.py --seed 7
"""
import argparse
import math
import random
import re
from pathlib import Path

WORLD = Path(__file__).resolve().parents[1] / "worlds" / "big_no_obstacles.wbt"
NAMES = [f"R{i}" for i in range(1, 6)]


def scatter(seed, extent, min_gap, tries=20000):
    """Poses at least min_gap apart, inside +/-extent, with random headings."""
    rng = random.Random(seed)
    placed = []
    for _ in range(tries):
        if len(placed) == len(NAMES):
            break
        x, y = rng.uniform(-extent, extent), rng.uniform(-extent, extent)
        if all(math.dist((x, y), (px, py)) >= min_gap for px, py, _ in placed):
            placed.append((x, y, rng.uniform(0, 2 * math.pi)))
    if len(placed) != len(NAMES):
        raise SystemExit(f"could not place {len(NAMES)} robots {min_gap}m apart "
                         f"within +/-{extent}m; loosen the constraints")
    return placed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--extent", type=float, default=2.6,
                    help="half-width of the area the robots start in")
    ap.add_argument("--min-gap", type=float, default=2.0,
                    help="minimum separation between any two robots")
    args = ap.parse_args()

    text = WORLD.read_text()
    poses = scatter(args.seed, args.extent, args.min_gap)
    for name, (x, y, yaw) in zip(NAMES, poses):
        pattern = (r'(SimpleDiffRobot \{\s*\n)(\s*translation [^\n]*\n)'
                   r'(\s*rotation [^\n]*\n)?(\s*name "%s")' % name)
        m = re.search(pattern, text)
        if not m:
            raise SystemExit(f"{name} not found in {WORLD.name}")
        text = text[:m.start()] + (
            m.group(1) + f"  translation {x:.4f} {y:.4f} 0.085\n"
            f"  rotation 0 0 1 {yaw:.4f}\n" + m.group(4)) + text[m.end():]
    WORLD.write_text(text)

    print(f"seed {args.seed}: scattered within +/-{args.extent} m, "
          f"min gap {args.min_gap} m")
    for name, (x, y, yaw) in zip(NAMES, poses):
        print(f"  {name}: ({x:+.3f}, {y:+.3f})  heading {math.degrees(yaw):6.1f} deg")
    gaps = [math.dist(a[:2], b[:2]) for i, a in enumerate(poses)
            for b in poses[i + 1:]]
    print(f"  closest pair {min(gaps):.2f} m, furthest {max(gaps):.2f} m")


if __name__ == "__main__":
    main()
