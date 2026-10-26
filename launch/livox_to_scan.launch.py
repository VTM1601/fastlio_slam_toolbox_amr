"""Slice a point cloud by height in base_frame and flatten it to a 360 deg LaserScan.

Default input is FAST-LIO's motion-compensated /cloud_registered_body (frame body), which
the TF body -> base_link from fastlio_odom.launch.py brings into base_frame.
base_frame must sit on the floor: min_height keeps the floor out of the scan.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def config(name, value_type):
        # Typed, so min_height:=0 or range_max:=30 are not rejected as integers
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_to_scan',
        output='screen',
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
        parameters=[{
            'target_frame': LaunchConfiguration('base_frame'),
            'transform_tolerance': 0.01,
            'min_height': config('min_height', float),
            'max_height': config('max_height', float),
            'angle_min': -3.14159,  # -M_PI
            'angle_max': 3.14159,  # M_PI
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.1,  # Livox publish_freq 10 Hz
            # Livox reports (0,0,0) for no return -> range ~0, dropped by range_min
            'range_min': config('range_min', float),
            'range_max': config('range_max', float),
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': config('use_sim_time', bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/cloud_registered_body',
                              description='Input PointCloud2'),
        DeclareLaunchArgument('scan_topic', default_value='/livox/scan',
                              description='Output LaserScan'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Frame the height slice is taken in, on the floor'),
        # Measured floor returns: z = 0 +- 0.04 m in base_link on the reference AMR
        DeclareLaunchArgument('min_height', default_value='0.05',
                              description='Lowest point kept, metres above base_frame'),
        DeclareLaunchArgument('max_height', default_value='1.0',
                              description='Highest point kept, metres above base_frame'),
        DeclareLaunchArgument('range_min', default_value='0.1'),
        DeclareLaunchArgument('range_max', default_value='30.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        pointcloud_to_laserscan,
    ])
