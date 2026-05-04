from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    controller_config_file = os.path.join(get_package_share_directory('alphabot2'), 'robot_controller.yaml')
    joy_config_file = os.path.join(get_package_share_directory('alphabot2'), 'gamepad.yaml')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["src/alphabot2/sdf/alphabot2/alphabot2.sdf"]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller_alphabot2"],
    )
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_controller_alphabot2/cmd_vel

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
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        diff_controller,

        # joystick
        joy_node,
        teleop_node

    ])

