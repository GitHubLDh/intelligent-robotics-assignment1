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

## One-time setup (every team member does this on their own machine)

These are personal machine installs — not part of this repo, not shareable:

1. Install **ROS2 Jazzy** (Ubuntu 24.04): https://docs.ros.org/en/jazzy/Installation.html
2. Install **Webots**: https://cyberbotics.com/
3. Install the ROS2–Webots bridge:
   ```bash
   sudo apt install ros-jazzy-webots-ros2
   ```
4. Make sure you can run:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch webots_ros2_epuck robot_launch.py
   ```
   and see Webots open with a robot in it. If this doesn't work, fix it before touching this repo — see Moodle's ROS2/Gazebo troubleshooting PDFs.

## Getting the project onto your machine

Clone this repo wherever you like on your machine — there's no required location.

```bash
git clone git@github.com:GitHubLDh/intelligent-robotics-assignment1.git
```

`colcon` (the ROS2 build tool) only looks for packages inside `~/ros2_ws/src/`,
but your clone can live anywhere. So the one thing you need is a **symlink**
inside `~/ros2_ws/src/` that points at the `reactive_following` folder in
*your* clone — this makes colcon see the package without copying any files:

```bash
mkdir -p ~/ros2_ws/src
ln -s /absolute/path/to/your/clone/ros2_ws/src/reactive_following ~/ros2_ws/src/reactive_following
```

Replace `/absolute/path/to/your/clone` with wherever you actually cloned the
repo (run `pwd` inside it to get the exact path). Already have a `ros2_ws`
elsewhere, or use a different workspace name entirely? Fine — just symlink
into whatever `src/` folder your `colcon build` command targets.

Build it:
```bash
cd ~/ros2_ws   # or wherever your workspace is
colcon build --packages-select reactive_following
source install/setup.bash
```

## Running

TODO — fill in once the first launch file exists, e.g.:
```bash
ros2 launch reactive_following five_robots_launch.py
```

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
- TODO: add teammates
