"""Pan axis alone, for bench work.

Brings up nothing that can move the car. Use it to check the servo, set
`invert` and `centre_offset_deg` against the real mount, and watch the axis
respond to hand-published angles:

    ros2 launch doggobot pan.launch.py
    ros2 topic pub -1 /pan_cmd std_msgs/msg/Float32 '{data: 30.0}'
    ros2 topic echo /pan_state

For a servo with no ROS at all, tools/pan_console.py talks to the firmware
directly and has a self-test that reports measured-vs-commanded error.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('doggobot')
    return LaunchDescription([
        Node(package='doggobot', executable='pan_node', name='pan_node',
             parameters=[os.path.join(share, 'config', 'pan.yaml')],
             output='screen', emulate_tty=True),
    ])
