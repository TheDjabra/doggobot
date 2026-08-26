"""Everything: perception, follow, phone control, arbiter, actuator.

This is the launch to use for GROUND testing, because it is the only one that
brings up both the autonomous loop and the kill switch. `follow.launch.py` omits
the phone bridge and is bench-only for that reason.

    vesc_twist_node     class actuator
    arbiter_node        sole /cmd_vel publisher
    perception_node     /target_state
    follow_node         /behavior_cmd
    voice_bridge_node   phone: sticks, kill switch, (later) speech

Arbiter priority means the phone always wins: e-stop beats everything, and the
sticks outrank the follow controller, so grabbing a stick takes the car off the
autonomous loop instantly without stopping any node.

Nothing follows until a lock is requested:

    ros2 topic pub -1 /target_lock std_msgs/msg/String '{data: "{\\"action\\":\\"lock\\"}"}'
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
        node('voice_bridge_node', 'bridge.yaml'),
    ])
