from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    joy_config_file = os.path.join(get_package_share_directory('alphabot2'), 'gamepad.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1}]
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_file],
        remappings=[('/cmd_vel', '/diff_controller_alphabot2/cmd_vel')]
    )


    return LaunchDescription([
        joy_node,
        teleop_node,
    ])

