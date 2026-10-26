"""Mapping: FAST-LIO2 odometry (odom -> base_link) + slam_toolbox 2D mapping (map -> odom).

  ros2 launch livox_ros_driver2 msg_MID360_launch.py      # CustomMsg (xfer_format 1)
  ros2 launch fastlio_slam_toolbox_amr mapping.launch.py rviz:=true
  (keep the robot still for a few seconds while FAST-LIO initialises the IMU)
  ros2 run fastlio_slam_toolbox_amr save_map ~/maps/warehouse

Pipeline: /livox/lidar + /livox/imu -> FAST-LIO -> /cloud_registered_body
  -> livox_to_scan (height slice in base_link) -> /livox/scan -> slam_toolbox -> /map
Do not run another node that publishes map -> odom or odom -> base_link (e.g. Cartographer).
All arguments of the included launch files (lidar_pose, min_height, ...) can be set here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('fastlio_slam_toolbox_amr')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)))

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'base_frame': LaunchConfiguration('base_frame'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # Fixed frame map: /map, /livox/scan, robot, FAST-LIO cloud and path, slam_toolbox panel
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(share, 'rviz', 'fastlio_slam_toolbox.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share, 'config', 'slam_toolbox_mapping.yaml'),
            description='slam_toolbox parameters file'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz'),
        include('fastlio_odom.launch.py'),
        include('livox_to_scan.launch.py'),
        slam_toolbox,
        rviz,
    ])
