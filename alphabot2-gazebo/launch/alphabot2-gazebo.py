from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value="/ros2_ws/src/alphabot2-gazebo/sdf/"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={
                "gz_args": "src/alphabot2-gazebo/sdf/simulacia.sdf",
                # "gz_args": "-s -r src/alphabot2-gazebo/sdf/simulacia.sdf",
                "on_exit_shutdown": "True"
            }.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                       "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                       "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                       "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                       "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                       "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                       "/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                       # "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                       ],
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=["src/alphabot2-gazebo/sdf/alphabot2/alphabot2.sdf"],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', '/ros2_ws/alphabot2.rviz'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': '/ros2_ws/src/alphabot2-gazebo/launch/mapper_params_online_async.yaml',
            }.items()
        )

    ])
