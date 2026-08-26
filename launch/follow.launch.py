"""The complete follow stack: camera -> controller -> arbiter -> VESC.

    vesc_twist_node    class actuator
    arbiter_node       sole /cmd_vel publisher
    perception_node    /target_state
    follow_node        /behavior_cmd

Nothing follows until a lock is requested, which is deliberate:

    ros2 topic pub -1 /target_lock std_msgs/msg/String '{data: "{\\"action\\":\\"lock\\"}"}'

The phone stack is NOT included here. Bring up phone.launch.py instead when you
want manual override available, since teleop outranks behaviour in the arbiter.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('doggobot')
    vesc_launch = os.path.join(
        get_package_share_directory('ucsd_robocar_actuator2_pkg'),
        'launch', 'vesc_twist.launch.py')

    def node(exe, cfg):
        return Node(package='doggobot', executable=exe, name=exe,
                    parameters=[os.path.join(share, 'config', cfg)],
                    output='screen', emulate_tty=True)

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vesc_launch)),
        node('arbiter_node', 'arbiter.yaml'),
        node('perception_node', 'perception.yaml'),
        node('follow_node', 'follow.yaml'),
    ])
