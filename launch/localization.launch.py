"""Localization: FAST-LIO2 odometry (odom -> base_link) + slam_toolbox localization on a
saved pose graph (map -> odom, and /map latched for nav2, so no map_server is needed).

  ros2 launch fastlio_slam_toolbox_amr localization.launch.py \\
    map_file:=~/maps/warehouse start_pose:="1.2 -0.5 1.57" rviz:=true

map_file: pose graph saved by `ros2 run fastlio_slam_toolbox_amr save_map`, without .posegraph.
start_pose: "x y yaw" in the map frame. slam_toolbox loads no map without a start pose and
does not support starting "at dock", so the default "0 0 0" means "where mapping started".
Anywhere else: pass start_pose, or use "2D Pose Estimate" in RViz (/initialpose).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def localization_node(context):
    map_file = os.path.expanduser(LaunchConfiguration('map_file').perform(context))
    if map_file.endswith('.posegraph'):
        map_file = map_file[:-len('.posegraph')]
    if not os.path.isfile(map_file + '.posegraph'):
        return [LogInfo(msg=f'[localization] {map_file}.posegraph not found'),
                Shutdown(reason='map not found')]

    # "x y yaw", "x,y,yaw" or "[x, y, yaw]" -> floats; launch_ros would not cast [1, 2, 0]
    raw = LaunchConfiguration('start_pose').perform(context)
    try:
        start_pose = [float(v) for v in raw.translate(str.maketrans('[],', '   ')).split()]
    except ValueError:
        start_pose = []
    if len(start_pose) != 3:
        return [LogInfo(msg=f'[localization] start_pose "{raw}" must be "x y yaw"'),
                Shutdown(reason='bad start_pose')]

    return [
        LogInfo(msg=f'[localization] map {map_file}, start pose {start_pose}'),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'map_file_name': map_file,
                    'map_start_pose': start_pose,
                    'base_frame': LaunchConfiguration('base_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                   value_type=bool),
                },
            ],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('fastlio_slam_toolbox_amr')

    def include(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=launch_arguments.items())

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(share, 'rviz', 'fastlio_slam_toolbox.rviz')],
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                    value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file',
                              description='slam_toolbox pose graph, path without .posegraph'),
        DeclareLaunchArgument('start_pose', default_value='0 0 0',
                              description='Initial pose "x y yaw" in the map frame'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share, 'config', 'slam_toolbox_localization.yaml'),
            description='slam_toolbox parameters file'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz (2D Pose Estimate -> /initialpose)'),
        # Localization runs for hours: /path would grow without bound
        include('fastlio_odom.launch.py', publish_path='false'),
        include('livox_to_scan.launch.py'),
        OpaqueFunction(function=localization_node),
        rviz,
    ])
