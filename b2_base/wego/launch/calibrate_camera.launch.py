# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

"""B2 driver entry point for the selected camera's calibration tool."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network_interface', description='B2 Ethernet NIC; required.'),
        DeclareLaunchArgument(
            'board_size', description='Measured checkerboard inner corners NxM.'),
        DeclareLaunchArgument(
            'square_size_m', description='Measured square side length in meters.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('b2_camera_calibration'), 'launch', 'calibrate.launch.py',
        ])), launch_arguments={
            name: LaunchConfiguration(name)
            for name in ('network_interface', 'board_size', 'square_size_m')
        }.items()),
    ])
