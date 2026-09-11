from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
                # The legacy bridge speaks Unitree's generated CycloneDDS
                # messages directly.  The SDK2 node must not use this RMW:
                # SDK2 creates its own Cyclone participant in the same process.
                SetEnvironmentVariable(
                    "RMW_IMPLEMENTATION",
                    "rmw_cyclonedds_cpp",
                    condition=LaunchConfigurationEquals("transport", "ros_bridge"),
                ),
                SetEnvironmentVariable(
                    "CYCLONEDDS_URI",
                    "<CycloneDDS><Domain><General><Interfaces>"
                    f'<NetworkInterface name="{network_interface}" priority="default" multicast="default" />'
                    "</Interfaces></General></Domain></CycloneDDS>",
                    condition=LaunchConfigurationEquals("transport", "ros_bridge"),
                ),
            ]
        )

    # SDK2 has an embedded CycloneDDS participant.  Fast DDS keeps rclcpp's
    # ROS graph participant separate so both can coexist inside this process.
    actions.append(
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION",
            "rmw_fastrtps_cpp",
            condition=LaunchConfigurationEquals("transport", "sdk"),
        )
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
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("transport"), "' == 'ros_bridge' and '",
                    LaunchConfiguration("enable_control"), "' == 'true'",
                ])
            ),
        )
    )

    actions.append(
        Node(
            package="b2_base",
            executable="b2_state_bridge",
            name="b2_state_bridge",
            output="screen",
            parameters=[str(params_file), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("transport"), "' == 'ros_bridge' and '",
                    LaunchConfiguration("enable_bridge"), "' == 'true'",
                ])
            ),
        )
    )

    # Hardware default: Unitree's SDK2 owns DDS and talks to the B2 directly.
    # The former ROS/DDS bridge stays available as transport:=ros_bridge for
    # compatibility and diagnostic work, but never runs concurrently with SDK2.
    actions.append(
        Node(
            package="b2_base",
            executable="b2_sdk_driver",
            name="b2_sdk_driver",
            output="screen",
            parameters=[
                str(params_file),
                {
                    "network_interface": network_interface,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
            condition=LaunchConfigurationEquals("transport", "sdk"),
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
            DeclareLaunchArgument(
                "transport",
                default_value="sdk",
                choices=["sdk", "ros_bridge"],
                description="sdk uses Unitree SDK2 directly; ros_bridge uses the legacy ROS/DDS bridge.",
            ),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            DeclareLaunchArgument("enable_bridge", default_value="true"),
            DeclareLaunchArgument("enable_description", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
