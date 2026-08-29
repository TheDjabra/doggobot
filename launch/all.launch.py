"""Everything: perception, follow, phone control, arbiter, actuator.

This is the launch to use for GROUND testing, because it is the only one that
brings up both the autonomous loop and the kill switch. `follow.launch.py` omits
the phone bridge and is bench-only for that reason.

    vesc_twist_node     class actuator
    arbiter_node        sole /cmd_vel publisher
    perception_node     /target_state
    follow_node         /follow_cmd
    behavior_node       /behavior_cmd  (sole owner; relays follow_cmd in follow mode)
    stt_node            on-board mic -> /voice_cmd (offline Vosk)
    ldlidar             LD06 driver -> /scan
    safety_node         /scan -> /safety_cmd, the 290 degrees the camera cannot see
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('doggobot')

    # Swap the follow tuning without editing files, e.g. the timid first-run set:
    #   ros2 launch doggobot all.launch.py follow_config:=follow_firstrun.yaml
    follow_config = LaunchConfiguration('follow_config')
    vesc_launch = os.path.join(
        get_package_share_directory('ucsd_robocar_actuator2_pkg'),
        'launch', 'vesc_twist.launch.py')

    def node(exe, cfg):
        return Node(package='doggobot', executable=exe, name=exe,
                    parameters=[os.path.join(share, 'config', cfg)],
                    output='screen', emulate_tty=True)

    return LaunchDescription([
        DeclareLaunchArgument('follow_config', default_value='follow.yaml',
                              description='which follow tuning file to load'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vesc_launch)),
        node('arbiter_node', 'arbiter.yaml'),
        node('perception_node', 'perception.yaml'),
        Node(package='doggobot', executable='follow_node', name='follow_node',
             parameters=[PathJoinSubstitution([share, 'config', follow_config])],
             output='screen', emulate_tty=True),
        node('behavior_node', 'behavior.yaml'),
        node('stt_node', 'stt.yaml'),
        node('safety_node', 'safety.yaml'),
        # The LiDAR driver lives in the class framework's own workspace.
        Node(package='ldlidar', executable='ldlidar', name='ldlidar',
             parameters=[{'serial_port': '/dev/ttyUSB0', 'topic_name': 'scan',
                          'lidar_frame': 'laser', 'range_threshold': 0.005}],
             output='screen'),
        node('voice_bridge_node', 'bridge.yaml'),
    ])
