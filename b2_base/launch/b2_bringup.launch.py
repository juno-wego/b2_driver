from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *_args, **_kwargs):
    network_interface = LaunchConfiguration("network_interface").perform(context).strip()
    params_file = Path(LaunchConfiguration("params_file").perform(context))

    if not params_file.exists():
        raise FileNotFoundError(params_file)

    actions = []

    if network_interface:
        actions.extend(
            [
                SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
                SetEnvironmentVariable(
                    "CYCLONEDDS_URI",
                    "<CycloneDDS><Domain><General><Interfaces>"
                    f'<NetworkInterface name="{network_interface}" priority="default" multicast="default" />'
                    "</Interfaces></General></Domain></CycloneDDS>",
                ),
            ]
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("b2_driver_description"), "launch", "b2_description.launch.py"]
                )
            ),
            launch_arguments={
                "description_file": LaunchConfiguration("description_file"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "start_rviz": LaunchConfiguration("start_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_description")),
        )
    )

    actions.append(
        Node(
            package="b2_base",
            executable="b2_cmd_vel_bridge",
            name="b2_cmd_vel_bridge",
            output="screen",
            parameters=[str(params_file), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("enable_control")),
        )
    )

    actions.append(
        Node(
            package="b2_base",
            executable="b2_state_bridge",
            name="b2_state_bridge",
            output="screen",
            parameters=[str(params_file), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("enable_bridge")),
        )
    )

    return actions


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("b2_base"), "config", "b2_driver_params.yaml"]
    )
    default_description = PathJoinSubstitution(
        [FindPackageShare("b2_driver_description"), "urdf", "b2_description.urdf"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("b2_driver_description"), "launch", "check_joint.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("description_file", default_value=default_description),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("network_interface", default_value=""),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            DeclareLaunchArgument("enable_bridge", default_value="true"),
            DeclareLaunchArgument("enable_description", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
