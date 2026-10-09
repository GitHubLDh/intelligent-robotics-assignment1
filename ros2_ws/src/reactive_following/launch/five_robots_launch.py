import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

# name, namespace, base linear speed (m/s), base angular speed (rad/s) -
# each robot's individual "base curve" when nothing is detected.
ROBOTS = [
    ('R1', 'r1', 0.05, 0.6),
    ('R2', 'r2', 0.04, -0.4),
    ('R3', 'r3', 0.06, 0.3),
    ('R4', 'r4', 0.05, -0.6),
    ('R5', 'r5', 0.045, 0.45),
]


def generate_launch_description():
    package_dir = get_package_share_directory('reactive_following')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')

    controllers = []
    for name, namespace, base_v, base_w in ROBOTS:
        controllers.append(WebotsController(
            robot_name=name,
            namespace=namespace,
            parameters=[
                {'robot_description': robot_description_path},
                {'base_linear_speed': base_v},
                {'base_angular_speed': base_w},
                # TODO: once reactive_driver.py grows parameters for the
                # turning behaviour (e.g. a gain), declare + pass them here too.
            ],
            respawn=True,
        ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='arena.wbt',
            description='World file from the reactive_following/worlds directory'
        ),
        webots,
        webots._supervisor,
        *controllers,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
