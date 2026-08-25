"""Arbiter plus the VESC actuator: the minimum stack that makes the car move.

Starts the class `vesc_twist_node` (loading the car's own
ros_racer_calibration.yaml) and our arbiter, and nothing else. With this
running, a single `ros2 topic pub` to /behavior_cmd or /teleop_cmd drives the
car.

NOT compatible with the class `all_nodes.launch.py`: that starts
lane_guidance_node, which also publishes /cmd_vel. Two publishers on that topic
is exactly the race the arbiter exists to prevent. Run one or the other.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    arbiter_config = os.path.join(
        get_package_share_directory('doggobot'), 'config', 'arbiter.yaml')

    vesc_launch = os.path.join(
        get_package_share_directory('ucsd_robocar_actuator2_pkg'),
        'launch', 'vesc_twist.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vesc_launch)),
        Node(
            package='doggobot',
            executable='arbiter_node',
            name='arbiter_node',
            parameters=[arbiter_config],
            output='screen',
            emulate_tty=True,
        ),
    ])
