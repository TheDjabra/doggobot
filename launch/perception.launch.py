"""Perception on its own: camera to /target_state.

Deliberately separate from the drive stack so the follow loop's two halves can be
brought up and debugged independently. Combine with drive.launch.py when the
follow controller exists.

    ros2 topic echo /target_state
    ros2 topic pub -1 /target_lock std_msgs/msg/String '{data: "{\\"action\\":\\"lock\\"}"}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('doggobot'), 'config', 'perception.yaml')

    return LaunchDescription([
        Node(package='doggobot', executable='perception_node',
             name='perception_node', parameters=[config],
             output='screen', emulate_tty=True),
    ])
