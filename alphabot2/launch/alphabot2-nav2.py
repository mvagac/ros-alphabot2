from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    map_file = os.path.join(get_package_share_directory('alphabot2'), 'kancel_map.yaml')
    nav2_config_file = os.path.join(get_package_share_directory('alphabot2'), 'nav2_params.yaml')

    map_server = Node(
        # map is published only once; in Rviz2 change Durability Policy to "Transient Local"
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': False}]
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': False}]
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config_file]
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config_file],
        remappings=[('/cmd_vel', '/diff_controller_alphabot2/cmd_vel')]
    )
    nav2_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_file]
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config_file]
    )
    bt_navigator = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'bond_timeout': 60.0},
                    {'node_names': ['map_server', 
                                    'amcl', 
                                    'planner_server', 
                                    'controller_server', 
                                    'behavior_server', 
                                    'bt_navigator']}]
    )

    return LaunchDescription([
        map_server,
        amcl,
        planner_server,
        controller_server,
        nav2_bt_nav,
        behavior_server,
        bt_navigator,
    ])

