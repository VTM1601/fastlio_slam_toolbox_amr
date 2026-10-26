"""FAST-LIO2 as the robot odometry: TF odom -> base_link and /cloud_registered_body.

Included by mapping.launch.py and localization.launch.py; can also run on its own.
The Livox driver must publish livox_ros_driver2/CustomMsg (xfer_format 1), e.g.
  ros2 launch livox_ros_driver2 msg_MID360_launch.py

TF tree:
  odom -(static)-> camera_init -(FAST-LIO)-> body -(static)-> base_link
FAST-LIO hard-codes camera_init (the IMU pose at start-up) and body (the IMU now).
Both static transforms are base_link -> IMU (and its inverse), computed from
  lidar_pose               base_link -> lidar frame, i.e. the mount pose in the URDF
  extrinsic_T/extrinsic_R  lidar -> IMU from the FAST-LIO config
so odom is base_link at start-up and the robot pose is reported for base_link.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll), the URDF / tf2 convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_to_rpy(r):
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    return math.atan2(r[2][1], r[2][2]), pitch, math.atan2(r[1][0], r[0][0])


def compose(a, b):
    """(R, t) of frame b expressed through frame a: a * b."""
    ra, ta = a
    rb, tb = b
    r = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [sum(ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3)]
    return r, t


def invert(a):
    r, t = a
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    return rt, [-sum(rt[i][k] * t[k] for k in range(3)) for i in range(3)]


def static_tf(name, transform, parent, child):
    (x, y, z), (roll, pitch, yaw) = transform[1], matrix_to_rpy(transform[0])
    values = {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
    args = []
    for key, value in values.items():
        args += [f'--{key}', f'{value:.6f}']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=args + ['--frame-id', parent, '--child-frame-id', child],
    )


def launch_setup(context):
    config = os.path.expanduser(LaunchConfiguration('fastlio_config').perform(context))
    base_frame = LaunchConfiguration('base_frame').perform(context)
    raw_pose = LaunchConfiguration('lidar_pose').perform(context)
    try:
        lidar_pose = [float(v) for v in raw_pose.replace(',', ' ').split()]
    except ValueError:
        lidar_pose = []
    if len(lidar_pose) != 6:
        return [LogInfo(msg=f'[fastlio_odom] lidar_pose "{raw_pose}" '
                            'must be "x y z roll pitch yaw"'),
                Shutdown(reason='bad lidar_pose')]

    with open(config) as f:
        mapping = yaml.safe_load(f)['/**']['ros__parameters']['mapping']
    rot = mapping['extrinsic_R']
    # FAST-LIO extrinsic = pose of the lidar in the IMU frame (p_imu = R * p_lidar + T)
    imu_to_lidar = ([rot[0:3], rot[3:6], rot[6:9]], list(mapping['extrinsic_T']))
    base_to_lidar = (rpy_to_matrix(*lidar_pose[3:]), lidar_pose[:3])
    base_to_imu = compose(base_to_lidar, invert(imu_to_lidar))

    xyz = ' '.join(f'{v:.5f}' for v in base_to_imu[1])
    rpy = ' '.join(f'{v:.5f}' for v in matrix_to_rpy(base_to_imu[0]))
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        LogInfo(msg=f'[fastlio_odom] {base_frame} -> IMU: xyz {xyz}, rpy {rpy}'),
        # odom = base_link at start-up, camera_init = IMU at start-up
        static_tf('odom_to_camera_init', base_to_imu, 'odom', 'camera_init'),
        static_tf('body_to_base_link', invert(base_to_imu), 'body', base_frame),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'publish.path_en': ParameterValue(LaunchConfiguration('publish_path'),
                                                  value_type=bool),
            }],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('fastlio_slam_toolbox_amr')
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_pose', default_value='0 0 0.357 -0.0048 0.0132 0',
            description='Lidar mount "x y z roll pitch yaw" in base_frame (metres, radians)'),
        DeclareLaunchArgument(
            'fastlio_config', default_value=os.path.join(share, 'config', 'fastlio_mid360.yaml'),
            description='FAST-LIO parameters file (lidar_type, topics, lidar-IMU extrinsic)'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Robot base frame, on the floor'),
        DeclareLaunchArgument(
            'publish_path', default_value='true',
            description='Publish /path (grows with every pose, off for long runs)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
