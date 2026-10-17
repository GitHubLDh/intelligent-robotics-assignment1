"""Write one of five fixed starting layouts into the world file.

Slide 7 asks for identical initial conditions within a comparison and
different positions and headings across repeats. These five are fixed rather
than random so a given number always reproduces the same run exactly, and
they are chosen to exercise different parts of the controller:

  1 ring        five on a 0.95 m circle, tangent headings. Spacing is inside
                following range, so every robot starts already following the
                next one. The easy case, and the one to use when measuring
                steady-state following rather than acquisition.
  2 wide-ring   the same circle at 1.50 m. Spacing now exceeds following
                range, so the robots must APPROACH and close in first. Tests
                acquisition without also testing search.
  3 scattered   far apart, random-looking headings, minimum 2 m separation.
                Nothing is in view at the start, so this tests search, and it
                is the layout the formation timings were taken from.
  4 column      already in single file, all facing the same way. Tests whether
                a formed train is maintained rather than whether one forms.
  5 facing-in   on a circle, every robot pointed at the centre. Worst case:
                each sees another's red front half, so the avoidance rules all
                fire at once. Tests whether they disperse or deadlock.

    python3 tools/set_start_layout.py 3
"""
import argparse
import math
import re
from pathlib import Path

WORLD = Path(__file__).resolve().parents[1] / "worlds" / "big_no_obstacles.wbt"
NAMES = [f"R{i}" for i in range(1, 6)]
N = len(NAMES)


def ring(radius):
    """Evenly spaced, each heading along the tangent (anticlockwise)."""
    out = []
    for i in range(N):
        a = 2 * math.pi * i / N
        out.append((radius * math.cos(a), radius * math.sin(a),
                    (a + math.pi / 2) % (2 * math.pi)))
    return out


def facing_in(radius):
    """Evenly spaced, each pointed at the centre of the circle."""
    out = []
    for i in range(N):
        a = 2 * math.pi * i / N
        out.append((radius * math.cos(a), radius * math.sin(a),
                    (a + math.pi) % (2 * math.pi)))
    return out


def column(spacing):
    """Single file along +x, all facing +x. Highest x is the leader."""
    first = -spacing * (N - 1) / 2.0
    return [(first + i * spacing, 0.0, 0.0) for i in range(N)]


def scattered(seed, extent=2.6, min_gap=2.0, tries=20000):
    """Well-separated poses with random headings, reproducible per seed."""
    import random
    rng = random.Random(seed)
    placed = []
    for _ in range(tries):
        if len(placed) == N:
            break
        x, y = rng.uniform(-extent, extent), rng.uniform(-extent, extent)
        if all(math.dist((x, y), (px, py)) >= min_gap for px, py, _ in placed):
            placed.append((x, y, rng.uniform(0, 2 * math.pi)))
    if len(placed) != N:
        raise SystemExit("could not place all robots; loosen the constraints")
    return placed


LAYOUTS = {
    1: ("ring",       "0.95 m circle, tangent - starts already following",
        lambda: ring(0.95)),
    2: ("wide-ring",  "1.50 m circle, tangent - must approach first",
        lambda: ring(1.50)),
    3: ("scattered",  "far apart, random headings - must search",
        lambda: scattered(7)),
    4: ("column",     "single file, all facing +x - already a train",
        lambda: column(1.10)),
    5: ("facing-in",  "1.20 m circle, all pointed inward - worst case",
        lambda: facing_in(1.20)),
}


def apply(poses, world=WORLD):
    text = world.read_text()
    for name, (x, y, yaw) in zip(NAMES, poses):
        pattern = (r'(SimpleDiffRobot \{\s*\n)(\s*translation [^\n]*\n)'
                   r'(\s*rotation [^\n]*\n)?(\s*name "%s")' % name)
        m = re.search(pattern, text)
        if not m:
            raise SystemExit(f"{name} not found in {world.name}")
        text = text[:m.start()] + (
            m.group(1) + f"  translation {x:.4f} {y:.4f} 0.085\n"
            f"  rotation 0 0 1 {yaw:.4f}\n" + m.group(4)) + text[m.end():]
    world.write_text(text)


def main():
    ap = argparse.ArgumentParser(
        description="Select one of five fixed starting layouts.")
    ap.add_argument("layout", type=int, choices=sorted(LAYOUTS),
                    help="; ".join(f"{k}={v[0]}" for k, v in sorted(LAYOUTS.items())))
    args = ap.parse_args()

    name, description, build = LAYOUTS[args.layout]
    poses = build()
    apply(poses)

    print(f"layout {args.layout} ({name}): {description}")
    for robot, (x, y, yaw) in zip(NAMES, poses):
        print(f"  {robot}: ({x:+.3f}, {y:+.3f})  heading {math.degrees(yaw):6.1f} deg")
    gaps = [math.dist(a[:2], b[:2]) for i, a in enumerate(poses)
            for b in poses[i + 1:]]
    print(f"  closest pair {min(gaps):.2f} m, furthest {max(gaps):.2f} m")


if __name__ == "__main__":
    main()
