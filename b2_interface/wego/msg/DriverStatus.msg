# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

std_msgs/Header header
bool sport_state_fresh
bool low_state_fresh
uint32 sport_error_code
bool controller_fresh
bool controller_ready
bool controller_ready_required
bool motion_ready
string reason
string controller_form
string controller_name
int32 check_mode_result
int32 service_list_result
string[] service_names
int32[] service_status
int32[] service_protect
int32 version_result
string package_version
string[] module_names
string[] module_versions
