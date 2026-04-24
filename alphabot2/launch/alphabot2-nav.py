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
        ),

        # LDROBOT LiDAR publisher node
        Node(
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
        ),
        Node(
          package='tf2_ros',
          executable='static_transform_publisher',
          name='base_link_to_base_laser_ld06',
          arguments=['0','0','0.18','0','0','0','telo','base_laser']
        ),

        Node(
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
              'freq': 2.0
          }]
        )
        # ros2 topic pub --once /base_pose_ground_truth nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}, child_frame_id: 'telo'}"
        # ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args -p laser_scan_topic:=/scan -p odom_topic:=/odom -p base_frame_id:=telo -p odom_frame_id:=odom  -p freq:=2.0 --log-level debug

    ])

