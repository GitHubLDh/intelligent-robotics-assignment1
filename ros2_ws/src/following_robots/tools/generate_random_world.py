"""Generate a random Webots world for the five robots.

The output path is fixed on purpose: running this command again replaces
``worlds/random_world.wbt``.

Examples:
    python3 tools/generate_random_world.py
    python3 tools/generate_random_world.py --seed 7 --objects 5
"""
import argparse
import math
import random
from pathlib import Path


WORLD_PATH = Path(__file__).resolve().parents[1] / "worlds" / "random_world.wbt"
ROBOT_NAMES = [f"R{i}" for i in range(1, 6)]
ARENA_SIZE = 12.0
ROBOT_Z = 0.085
ROBOT_CLEARANCE = 0.45
COLORS = [
    (0.10, 0.72, 0.95), (0.95, 0.25, 0.20), (0.35, 0.85, 0.30),
    (0.95, 0.65, 0.12), (0.72, 0.30, 0.85),
]
OBJECT_COLORS = [
    (0.86, 0.34, 0.12), (0.15, 0.62, 0.42), (0.90, 0.68, 0.12),
    (0.55, 0.25, 0.72), (0.20, 0.48, 0.78),
]


def distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def choose_poses(rng):
    poses = []
    for _ in range(20000):
        if len(poses) == len(ROBOT_NAMES):
            return poses
        candidate = (rng.uniform(-4.75, 4.75), rng.uniform(-4.75, 4.75),
                     rng.uniform(0.0, 2.0 * math.pi))
        if all(distance(candidate, pose) >= 1.05 for pose in poses):
            poses.append(candidate)
    raise RuntimeError("could not place five robots with the requested gaps")


def choose_objects(rng, poses, count):
    objects = []
    for _ in range(count):
        for _ in range(20000):
            kind = rng.choice(("cylinder", "box"))
            if kind == "cylinder":
                radius = rng.uniform(0.25, 0.62)
                size_x = size_y = 2.0 * radius
                footprint = radius
            else:
                size_x = rng.uniform(0.55, 1.45)
                size_y = rng.uniform(0.45, 1.20)
                footprint = math.hypot(size_x, size_y) / 2.0
            height = rng.uniform(0.24, 0.80)
            x = rng.uniform(-4.65 + footprint, 4.65 - footprint)
            y = rng.uniform(-4.65 + footprint, 4.65 - footprint)
            candidate = {
                "kind": kind, "x": x, "y": y, "height": height,
                "size_x": size_x, "size_y": size_y, "footprint": footprint,
                "yaw": rng.uniform(-math.pi, math.pi),
                "color": rng.choice(OBJECT_COLORS),
            }
            if any(distance((x, y), pose) < footprint + ROBOT_CLEARANCE
                   for pose in poses):
                continue
            if any(distance((x, y), (other["x"], other["y"]))
                   < footprint + other["footprint"] + 0.20
                   for other in objects):
                continue
            objects.append(candidate)
            break
        else:
            raise RuntimeError("could not place all random objects")
    return objects


def color_text(color):
    return " ".join(f"{value:.3f}" for value in color)


def object_text(index, obstacle):
    name = f"random_object_{index}"
    color = color_text(obstacle["color"])
    translation = (f"  translation {obstacle['x']:.4f} {obstacle['y']:.4f} "
                   f"{obstacle['height'] / 2:.4f}\n")
    if obstacle["kind"] == "cylinder":
        geometry = (f"Cylinder {{ radius {obstacle['size_x'] / 2:.4f} "
                    f"height {obstacle['height']:.4f} subdivision 24 }}")
    else:
        geometry = (f"Box {{ size {obstacle['size_x']:.4f} "
                    f"{obstacle['size_y']:.4f} {obstacle['height']:.4f} }}")
    return f'''Solid {{
{translation}  rotation 0 0 1 {obstacle['yaw']:.4f}
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color} roughness 0.85 }}
      geometry {geometry}
    }}
  ]
  boundingObject {geometry}
}}
'''


def robot_text(name, pose, color):
    x, y, yaw = pose
    return ("SimpleDiffRobot {\n"
            f"  translation {x:.4f} {y:.4f} {ROBOT_Z:.3f}\n"
            f"  rotation 0 0 1 {yaw:.4f}\n"
            f"  name \"{name}\"\n"
            "  controller \"udp_diff_driver\"\n"
            f"  bodyColor {color_text(color)}\n"
            "}\n")


def world_text(poses, objects, seed, object_count):
    objects_text = "\n".join(object_text(i, obstacle)
                                 for i, obstacle in enumerate(objects, 1))
    robots_text = "\n".join(robot_text(name, pose, color)
                                for name, pose, color in zip(ROBOT_NAMES, poses, COLORS))
    return f'''#VRML_SIM R2023b utf8
EXTERNPROTO "../protos/SimpleDiffRobot.proto"

WorldInfo {{
  title "Random Reactive Following World (seed {seed})"
  basicTimeStep 32
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties {{ material1 "body_support" material2 "floor" coulombFriction [ 0.05 ] bounce 0 }}
    ContactProperties {{ material1 "wheel_rubber" material2 "floor" coulombFriction [ 1.15 ] bounce 0 }}
    ContactProperties {{ material1 "body_support" material2 "default" coulombFriction [ 0.18 ] bounce 0 }}
    ContactProperties {{ material1 "wheel_rubber" material2 "default" coulombFriction [ 0.6 ] bounce 0 }}
    ContactProperties {{ material1 "body_support" material2 "body_support" coulombFriction [ 0.18 ] bounce 0 }}
    ContactProperties {{ material1 "wheel_rubber" material2 "body_support" coulombFriction [ 0.35 ] bounce 0 }}
    ContactProperties {{ material1 "wheel_rubber" material2 "wheel_rubber" coulombFriction [ 0.45 ] bounce 0 }}
  ]
}}

Viewpoint {{ orientation 1 0 0 0.82 position 0 -14 15 }}
Background {{ skyColor [ 0.67 0.75 0.83 ] }}
DirectionalLight {{ direction 0.35 0.50 -1 intensity 1.5 ambientIntensity 0.4 }}

Solid {{
  translation 0 0 -0.05
  name "floor"
  contactMaterial "floor"
  children [Shape {{ appearance PBRAppearance {{ baseColor 0.19 0.23 0.27 roughness 1 metalness 0 }} geometry DEF FLOOR_BOX Box {{ size 12 12 0.1 }} }}]
  boundingObject USE FLOOR_BOX
}}
Solid {{ translation 0 5.94 0.12 name "north_wall"
  children [Shape {{ appearance PBRAppearance {{ baseColor 0.60 0.65 0.70 roughness 0.95 }} geometry DEF WALL_N Box {{ size 12 0.12 0.24 }} }}]
  boundingObject USE WALL_N }}
Solid {{ translation 0 -5.94 0.12 name "south_wall"
  children [Shape {{ appearance PBRAppearance {{ baseColor 0.60 0.65 0.70 roughness 0.95 }} geometry DEF WALL_S Box {{ size 12 0.12 0.24 }} }}]
  boundingObject USE WALL_S }}
Solid {{ translation 5.94 0 0.12 name "east_wall"
  children [Shape {{ appearance PBRAppearance {{ baseColor 0.60 0.65 0.70 roughness 0.95 }} geometry DEF WALL_E Box {{ size 0.12 11.76 0.24 }} }}]
  boundingObject USE WALL_E }}
Solid {{ translation -5.94 0 0.12 name "west_wall"
  children [Shape {{ appearance PBRAppearance {{ baseColor 0.60 0.65 0.70 roughness 0.95 }} geometry DEF WALL_W Box {{ size 0.12 11.76 0.24 }} }}]
  boundingObject USE WALL_W }}

# Random objects: {object_count}
{objects_text}

# Randomized robot starts: seed {seed}
{robots_text}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=None,
                        help="seed for repeatable output; omit for a new layout")
    parser.add_argument("--objects", type=int, choices=range(6), default=None,
                        help="number of objects from 0 to 5; omit for random")
    args = parser.parse_args()
    rng = random.Random(args.seed)
    seed = args.seed if args.seed is not None else rng.randrange(1 << 32)
    rng.seed(seed)
    object_count = args.objects if args.objects is not None else rng.randint(0, 5)
    poses = choose_poses(rng)
    objects = choose_objects(rng, poses, object_count)
    WORLD_PATH.write_text(world_text(poses, objects, seed, object_count), encoding="utf-8")
    print(f"wrote {WORLD_PATH}")
    print(f"seed={seed} arena={ARENA_SIZE:.1f}x{ARENA_SIZE:.1f}m objects={object_count}")
    for name, (x, y, yaw) in zip(ROBOT_NAMES, poses):
        print(f"{name}: x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):.1f}deg")


if __name__ == "__main__":
    main()