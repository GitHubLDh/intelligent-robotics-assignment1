"""Launch the Webots UDP bridge and the 5 autonomous reactive follower nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reactive_following',
            executable='bridge',
            output='screen',
        ),
        Node(
            package='reactive_following',
            executable='follower',
            output='screen',
        ),
    ])

