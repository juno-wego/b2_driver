from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    package_share = Path(get_package_share_directory("b2_base"))
    description_file = Path(LaunchConfiguration("description_file").perform(context))
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context))
    network_interface = LaunchConfiguration("network_interface").perform(context).strip()

    if not description_file.exists():
        raise FileNotFoundError(description_file)
    if not rviz_config.exists():
        raise FileNotFoundError(rviz_config)
    if not package_share.exists():
        raise FileNotFoundError(package_share)

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

    if LaunchConfiguration("enable_description").perform(context).lower() == "true":
        robot_description = description_file.read_text()
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            )
        )

    if LaunchConfiguration("enable_wait_mode").perform(context).lower() == "true":
        actions.append(
            ExecuteProcess(
                cmd=["bash", "-lc", "exec bash"],
                output="screen",
            )
        )

    if LaunchConfiguration("enable_control").perform(context).lower() == "true":
        actions.append(
            Node(
                package="b2_base",
                executable="b2_cmd_vel_bridge",
                name="b2_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "request_topic": LaunchConfiguration("request_topic"),
                        "max_linear_x": LaunchConfiguration("max_linear_x"),
                        "max_linear_y": LaunchConfiguration("max_linear_y"),
                        "max_angular_z": LaunchConfiguration("max_angular_z"),
                        "cmd_timeout": LaunchConfiguration("cmd_timeout"),
                        "auto_balance_stand_on_start": LaunchConfiguration("auto_balance_stand_on_start"),
                    }
                ],
            )
        )

    if LaunchConfiguration("enable_bridge").perform(context).lower() == "true":
        actions.append(
            Node(
                package="b2_base",
                executable="b2_state_bridge",
                name="b2_state_bridge",
                output="screen",
                parameters=[
                    {
                        "sport_state_topic": LaunchConfiguration("sport_state_topic"),
                        "low_state_topic": LaunchConfiguration("low_state_topic"),
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "imu_frame": LaunchConfiguration("imu_frame"),
                    }
                ],
            )
        )

    if LaunchConfiguration("start_rviz").perform(context).lower() == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    default_description = PathJoinSubstitution(
        [FindPackageShare("shalom_b2_description"), "urdf", "b2_description.urdf"]
    )
    default_rviz = PathJoinSubstitution([FindPackageShare("shalom_b2_description"), "rviz", "b2_description.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("description_file", default_value=default_description),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("network_interface", default_value=""),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("enable_control", default_value="false"),
            DeclareLaunchArgument("enable_bridge", default_value="false"),
            DeclareLaunchArgument("enable_description", default_value="false"),
            DeclareLaunchArgument("enable_wait_mode", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("sport_state_topic", default_value="/sportmodestate"),
            DeclareLaunchArgument("low_state_topic", default_value="/lowstate"),
            DeclareLaunchArgument("request_topic", default_value="/api/sport/request"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("max_linear_x", default_value="0.8"),
            DeclareLaunchArgument("max_linear_y", default_value="0.4"),
            DeclareLaunchArgument("max_angular_z", default_value="1.2"),
            DeclareLaunchArgument("cmd_timeout", default_value="0.5"),
            DeclareLaunchArgument("auto_balance_stand_on_start", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
