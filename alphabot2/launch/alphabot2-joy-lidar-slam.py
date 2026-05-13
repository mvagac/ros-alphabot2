from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    controller_config_file = os.path.join(get_package_share_directory('alphabot2'), 'robot_controller.yaml')
    joy_config_file = os.path.join(get_package_share_directory('alphabot2'), 'gamepad.yaml')
    mapper_config_file = os.path.join(get_package_share_directory('alphabot2'), 'mapper_params.yaml')
    map_file = os.path.join(get_package_share_directory('alphabot2'), 'kancel_map.yaml')
    nav2_config_file = os.path.join(get_package_share_directory('alphabot2'), 'nav2_params.yaml')

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

    ### LIDAR #############################################

    ldlidarD500 = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[
          {'product_name': 'LDLiDAR_LD06'},
          {'topic_name': 'scan'},
          {'frame_id': 'base_laser'},
          {'port_name': '/dev/ttyUSB0'},
          {'port_baudrate': 230400},
          {'laser_scan_dir': True},
          {'enable_angle_crop_func': False},
          {'angle_crop_min': 135.0},
          {'angle_crop_max': 225.0}
        ]
    )

    sllidarC1 = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'base_laser',
            'inverted': False,
            'angle_compensate': True
        }],
        output='screen'
    )

    lidar_tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='telo_to_base_laser_ld06',
      arguments=['0','0','0.05','0','0','0','telo','base_laser']
    )

    rf2o = Node(
      package='rf2o_laser_odometry',
      executable='rf2o_laser_odometry_node',
      name='rf2o_laser_odometry',
      output='screen',
      parameters=[{
          'laser_scan_topic': '/scan',
          'odom_topic': '/odom',
          'publish_tf': True,
          'base_frame_id': 'telo',
          'odom_frame_id': 'odom',
          #'init_pose_from_topic': '',
          'freq': 8.0
      }]
    )

    ### JOYSTICK #############################################

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

    mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={ 'slam_params_file': mapper_config_file }.items()
    )

    ### NAVIGATION #############################################

    map_server = Node(
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
        parameters=[nav2_config_file]
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
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        diff_controller,

        # joystick
        joy_node,
        teleop_node,

        # lidar publisher node
        #ldlidarD500,
        sllidarC1,
        lidar_tf,

        rf2o,
        # ros2 topic pub --once /base_pose_ground_truth nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}, child_frame_id: 'telo'}"

        mapper,
        # ros2 run nav2_map_server map_saver_cli -f pokus_map

        map_server,
        amcl,
        planner_server,
        controller_server,
        nav2_bt_nav,
        behavior_server,
        bt_navigator,
    ])

