"""Launch the command arbiter on its own.

Deliberately minimal: the arbiter is the one node that must be running for
anything else in this project to move the car, so it is useful to be able to
start it by itself and drive the car with `ros2 topic pub` alone.

Do NOT run this at the same time as the class `all_nodes.launch.py`.
`lane_guidance_node` publishes /cmd_vel too, and two publishers on that topic is
exactly the race this node exists to prevent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('doggobot'), 'config', 'arbiter.yaml')

    return LaunchDescription([
        Node(
            package='doggobot',
            executable='arbiter_node',
            name='arbiter_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
