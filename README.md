# Intelligent Robotics — Assignment 1: Reactive Following with Five Robots

Webots + ROS2 simulation of five differential-drive robots. Each robot follows
its own fixed curve until it detects a neighbour in front of it, then turns
toward the nearest one; it resumes its own curve once the neighbour is lost
from view. Purely reactive: current sensor reading only, no GPS, no global
localization, no memory of past targets.

## Repo layout

```
.
├── ros2_ws/src/reactive_following/   # the ROS2 package (ament_python)
│   ├── launch/                       # launch files
│   ├── worlds/                       # Webots .wbt world file(s)
│   ├── protos/                       # custom Webots robot/object PROTOs (if any)
│   ├── reactive_following/           # Python controller nodes
│   └── resource/
├── article/                          # IEEE-format paper (deliverable 2)
└── demo_video/                       # video script/notes (the actual .mp4 is NOT committed here — see below)
```


## Getting the project onto your machine

```bash
git clone git@github.com:GitHubLDh/intelligent-robotics-assignment1.git
cd intelligent-robotics-assignment1/ros2_ws
colcon build
source install/setup.bash
```

## Running

```bash
ros2 launch reactive_following five_robots_launch.py
```

Opens Webots with 5 e-puck robots, each running its own instance of the
controller in `reactive_following/reactive_driver.py`. Note: the actual
reactive-following logic in that file is still a stub (`step()` has a
`TODO`) — right now the robots just drive their own fixed curves and ignore
each other.

## Workflow / contributing

- Work on a branch per feature/person, open a PR into `main` when ready (or at minimum, `git pull` before you start working and `git push` as soon as something works, to avoid conflicts).
- Don't commit `build/`, `install/`, `log/`, or `__pycache__/` — already in `.gitignore`.
- Don't commit the final demo video (too large, and git is bad at binary diffs) — upload it to Drive/Moodle directly and link it here when ready.
- Keep paths in code/launch files relative to the package (use `get_package_share_directory('reactive_following')`), never hardcode a personal path like `/home/yourname/...`.

## Deliverables (per assignment spec)

1. **Code** — this repo, zipped at submission time, with this README.
2. **Article** — 3–4 pages, IEEE format, in `article/`.
3. **Demo video** — 1080p MP4, <100MB, student names + FEUP logo, linked here once uploaded.

## Team

- Lara Dehandschutter
- Lennert De Rouck
- Pedro Cancela
