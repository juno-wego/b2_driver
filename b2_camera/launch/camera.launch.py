# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

"""Read B2 video through the existing SDK; no gait or actuator clients."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context).strip()

    side = argument("camera_side")
    interface = argument("network_interface")
    url = argument("camera_info_url")
    require = argument("require_calibration").lower()
    if side not in ("front", "back"):
        raise RuntimeError("camera_side must be front or back")
    if not interface:
        raise RuntimeError("network_interface is required for the B2 camera SDK")
    if require not in ("true", "false"):
        raise RuntimeError("require_calibration must be true or false")
    if require == "true" and not url:
        raise RuntimeError("camera_info_url must point to measured B2 camera calibration")
    try:
        rate = float(argument("publish_rate_hz"))
        timeout = float(argument("rpc_timeout_s"))
        domain = int(argument("dds_domain"))
    except ValueError as exc:
        raise RuntimeError("camera rate, timeout and DDS domain must be numeric") from exc
    if (not math.isfinite(rate) or not 0 < rate <= 60 or
            not math.isfinite(1e9 / rate) or 1e9 / rate >= 2**63 - 1 or
            not math.isfinite(timeout) or not 0 < timeout <= 10):
        raise RuntimeError("publish_rate_hz must be 0..60 and rpc_timeout_s 0..10 (positive finite)")
    if not 0 <= domain <= 232:
        raise RuntimeError("dds_domain must be between 0 and 232")
    namespace = argument("namespace") or f"b2/{side}_camera"
    # These optical frames are fixed joints in b2_driver_description's URDF.
    frame = argument("frame_id") or {"front": "f_oc_link", "back": "r_oc_link"}[side]
    if frame.startswith("/") or any(char.isspace() for char in frame):
        raise RuntimeError("frame_id must be a valid camera optical frame without leading slash")
    return [Node(
        package="b2_camera", executable="b2_camera_node", name="b2_camera", namespace=namespace,
        output="screen", parameters=[{
            "network_interface": interface, "camera_side": side, "dds_domain": domain,
            "publish_rate_hz": rate, "rpc_timeout_s": timeout, "camera_info_url": url,
            "require_calibration": require == "true", "frame_id": frame, "use_sim_time": False,
        }],
        # SDK embeds CycloneDDS. Use ROS Fast DDS only for this node, without
        # replacing the caller's RMW environment for unrelated processes.
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "RMW_FASTRTPS_USE_QOS_FROM_XML": "1",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
            "FASTRTPS_DEFAULT_PROFILES_FILE": PathJoinSubstitution([
                FindPackageShare("b2_camera"), "config", "fastdds.xml",
            ]),
        },
        remappings=[("~/set_camera_info", "set_camera_info")],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("network_interface", description="Required Ethernet interface connected to the B2."),
        DeclareLaunchArgument("camera_side", default_value="front", choices=["front", "back"]),
        DeclareLaunchArgument("camera_info_url", default_value="", description="Measured calibration file:// or package:// URL."),
        DeclareLaunchArgument("require_calibration", default_value="true", description="false is only for calibration image collection."),
        DeclareLaunchArgument("dds_domain", default_value="0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("rpc_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("namespace", default_value="", description="Empty derives b2/<side>_camera."),
        DeclareLaunchArgument("frame_id", default_value="", description="Empty uses URDF optical frame f_oc_link (front) or r_oc_link (back)."),
        OpaqueFunction(function=_setup),
    ])
