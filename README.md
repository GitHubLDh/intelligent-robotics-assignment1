# Intelligent Robotics — Assignment 1: Reactive Following with Five Robots

Webots + ROS 2 simulation of five differential-drive robots that follow one
another. Each robot drives its own fixed base curve until its camera sees the
green rear half of another robot, then steers onto it; it returns to the base
curve as soon as that target leaves view. Purely reactive: one current sensor
snapshot per decision, no GPS, no global localization, no map, no memory of
past targets, and no communication between robots.

## Authors

| Name | Student number |
|---|---|
| Lara Dehandschutter | up202603432 |
| Lennert De Rouck | up202602215 |
| Pedro Cancela | up202400230 |

FEUP / Universidade do Porto

## Requirements

| Component | Version used |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| Webots | R2025a |
| Python | 3.12 |

Python dependencies are `rclpy`, `std_msgs`, `geometry_msgs` and
`sensor_msgs`, all of which ship with a standard ROS 2 Jazzy desktop install.
Nothing else needs to be installed via `pip`.

Install ROS 2 Jazzy (https://docs.ros.org/en/jazzy/Installation.html) and
Webots (https://cyberbotics.com/). The `webots` binary must be on your `PATH`.

## Directory structure

```
.
├── run_trains.sh                     # one-command launcher (Webots + bridge + controller)
├── ros2_ws/src/following_robots/
│   ├── worlds/                       # Webots .wbt worlds
│   │   ├── big_four_obstacles.wbt    #   12x12 m arena, four static obstacles (primary trial)
│   │   ├── big_no_obstacles.wbt      #   12x12 m empty arena
│   │   ├── random_world.wbt          #   overwritten by tools/generate_random_world.py
│   │   └── straight_line.wbt
│   ├── protos/SimpleDiffRobot.proto  # custom robot: camera, frontal LiDAR, red/green halves, bumper
│   ├── controllers/udp_diff_driver/  # Webots-side controller (one process per robot)
│   │   ├── udp_diff_driver.py        #   reads local devices, sends snapshot over UDP
│   │   ├── perception.py             #   camera recognition -> nearest red / nearest green
│   │   └── lidar_scan.py             #   merges the three LiDAR layers
│   ├── ros2_bridge/                  # UDP <-> ROS 2 bridge
│   │   ├── bridge_all.py             #   five independent bridge nodes
│   │   └── sensor_packet.py
│   ├── ros2_controller/              # the reactive controller (the "brain")
│   │   ├── reactive_follow.py        #   five independent ROS 2 nodes, one per robot
│   │   ├── reactive_logic.py         #   the five-rule reactive cascade (stateless)
│   │   └── sensor_snapshot.py
│   ├── config/robots.json            # per-robot speed and base-curve parameters
│   ├── tools/                        # world generation and diagnostics (see below)
│   └── tests/                        # legacy unit tests (see "Tests" — currently broken)
├── article/                          # IEEE-format paper (deliverable 2)
└── docs/                             # architecture diagram and figures used in the paper
```

## Compiling

**There is nothing to compile.** This project is plain Python run directly
against the system ROS 2 installation — it is not an ament/colcon package and
has no `package.xml`, so do **not** run `colcon build`. Webots launches its own
robot controllers automatically when a world is opened.

## Running

From the repository root:

```bash
./run_trains.sh big_four_obstacles.wbt
```

This starts all three pieces in order and stops them together on `Ctrl+C`:

1. Webots with the chosen world,
2. `ros2_bridge/bridge_all.py` (the UDP↔ROS 2 translator),
3. `ros2_controller/reactive_follow.py` (the five controller nodes).

The script sources whichever ROS 2 distribution is present under `/opt/ros/`,
so you do not need to `source` anything yourself first.

Other worlds:

```bash
./run_trains.sh                        # defaults to big_no_obstacles.wbt
./run_trains.sh random_world.wbt       # a generated random world
```

Output from all three processes is written to `logs/{webots,bridge,follower}.log`,
overwritten on each run.

### Fixed starting layouts

Passing a bare number 1–5 instead of a world name selects a predefined
starting layout. Note this rewrites the robot poses **inside
`big_no_obstacles.wbt`** before Webots opens it:

```bash
./run_trains.sh 3                      # scattered start, robots must search
```

| # | Layout | Purpose |
|---|---|---|
| 1 | ring | 0.95 m circle, tangent — already following |
| 2 | wide-ring | 1.50 m circle — must approach first |
| 3 | scattered | far apart, random headings — must search |
| 4 | column | single file — tests train maintenance |
| 5 | facing-in | pointed inward — worst case, all avoidance fires |

### Random worlds

```bash
cd ros2_ws/src/following_robots
python3 tools/generate_random_world.py               # random seed
python3 tools/generate_random_world.py --seed 7 --objects 5
```

This overwrites `worlds/random_world.wbt`, keeping the 12×12 m arena while
randomizing all five starting poses and placing 0–5 collision-free obstacles.
The same `--seed` reproduces the same layout. Then run it with
`./run_trains.sh random_world.wbt`.

### Other tools

| Script | Purpose |
|---|---|
| `tools/randomize_starts.py` | reseeds only the starting poses, world unchanged |
| `tools/set_start_layout.py` | writes one of the five fixed layouts (used by `run_trains.sh`) |
| `tools/status_dashboard.py` | live terminal view of all five robots' mode, *v* and *ω* |

## Tests

`tests/` contains unit tests from an earlier controller revision. **Ten of the
eleven files do not currently run**: they import `safe_logic` and
`follow_improved`, modules that were removed when the controller was
simplified to `reactive_logic.py`. They are retained only as a record of the
earlier development stages and are not part of the submitted system.

## Deliverables

1. **Code** — this repository.
2. **Article** — 4 pages, IEEE format, submitted separately.
3. **Demo video** — 1080p MP4, submitted separately.
