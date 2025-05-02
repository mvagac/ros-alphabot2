from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    test_controller = os.path.join(get_package_share_directory('alphabot2'), 'robot_controller.yaml')

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=["src/alphabot2/sdf/alphabot2/alphabot2.sdf"]
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[test_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            }
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_controller_alphabot2"],
        )

    ])

