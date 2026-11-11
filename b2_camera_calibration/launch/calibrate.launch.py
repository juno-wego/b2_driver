"""One selected B2 raw camera and one named monocular calibration GUI."""

from b2_camera_calibration.configuration import (
    board_settings, camera_identity, default_output_root, output_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ResetLaunchConfigurations,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context).strip()

    side = argument('camera_side')
    camera_name, namespace = camera_identity(side)
    size, square = board_settings(argument('board_size'), argument('square_size_m'))
    root = str(output_directory(argument('output_root')))
    model = argument('camera_model')
    if model not in ('fisheye', 'pinhole'):
        raise ValueError('camera_model must be fisheye or pinhole')
    interface = argument('network_interface')
    if not interface:
        raise ValueError('network_interface is required for the selected B2 camera')
    camera = GroupAction(
        scoped=True,
        actions=[
            ResetLaunchConfigurations(),
            SetEnvironmentVariable('ROS_HOME', root),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('b2_camera'), 'launch', 'camera.launch.py']
                    )
                ),
                launch_arguments={
                    'network_interface': interface,
                    'camera_side': side,
                    'camera_info_url': '',
                    'require_calibration': 'false',
                    'namespace': namespace.lstrip('/'),
                    **{
                        name: argument(name)
                        for name in ['dds_domain', 'publish_rate_hz', 'rpc_timeout_s']
                    },
                }.items(),
            ),
        ],
    )
    calibrator = Node(
        package='b2_camera_calibration',
        executable='cameracalibrator',
        namespace=namespace + '/calibration',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--camera-side',
            side,
            '--board-size',
            f'{size[0]}x{size[1]}',
            '--square-size-m',
            str(square),
            '--camera-model',
            model,
            '--output-root',
            root,
        ],
        remappings=[
            ('image', namespace + '/image_raw'),
            ('camera/set_camera_info', namespace + '/set_camera_info'),
        ],
        additional_env={
            'ROS_HOME': root,
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET',
            'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
            'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS',
            'FASTRTPS_DEFAULT_PROFILES_FILE': PathJoinSubstitution(
                [FindPackageShare('b2_camera'), 'config', 'fastdds.xml']
            ),
        },
    )
    close = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrator,
            on_exit=[EmitEvent(event=Shutdown(reason=camera_name + ' calibration finished'))],
        )
    )
    return [close, camera, calibrator]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('network_interface', description='B2 Ethernet NIC; required.'),
            DeclareLaunchArgument('camera_side', default_value='front', choices=['front', 'back']),
            DeclareLaunchArgument(
                'board_size', description='Measured checkerboard inner corners NxM.'
            ),
            DeclareLaunchArgument(
                'square_size_m', description='Measured square side length in meters.'
            ),
            DeclareLaunchArgument(
                'camera_model', default_value='fisheye', choices=['fisheye', 'pinhole']
            ),
            DeclareLaunchArgument('output_root', default_value=default_output_root()),
            DeclareLaunchArgument('dds_domain', default_value='0'),
            DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
            DeclareLaunchArgument('rpc_timeout_s', default_value='0.5'),
            OpaqueFunction(function=_setup),
        ]
    )
