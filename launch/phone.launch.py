"""Everything needed to drive the car from a phone.

  vesc_twist_node  (class framework, the actuator)
  arbiter_node     (sole /cmd_vel publisher)
  voice_bridge_node(serves the control page, publishes /teleop_cmd and /estop)

Expose it to the phone over HTTPS from the Pi host, not inside the container:

    tailscale serve --bg 8080

which publishes https://doggobot.<tailnet>.ts.net . HTTPS is not decoration: the
Web Speech API refuses to give a page a microphone on an insecure origin, so the
voice tab cannot ever work over plain http://<ip>.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('doggobot')
    arbiter_config = os.path.join(share, 'config', 'arbiter.yaml')
    bridge_config = os.path.join(share, 'config', 'bridge.yaml')

    vesc_launch = os.path.join(
        get_package_share_directory('ucsd_robocar_actuator2_pkg'),
        'launch', 'vesc_twist.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vesc_launch)),
        Node(package='doggobot', executable='arbiter_node', name='arbiter_node',
             parameters=[arbiter_config], output='screen', emulate_tty=True),
        Node(package='doggobot', executable='voice_bridge_node',
             name='voice_bridge_node', parameters=[bridge_config],
             output='screen', emulate_tty=True),
    ])
