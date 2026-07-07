# b2_base

ROS 2 package for Unitree B2 state visualization and high-level velocity control.

## Build

```bash
cd /home/juno/ros2_ws
colcon build --packages-select b2_base
source install/setup.bash
```

## Launch

Use the Ethernet interface connected to the robot:

```bash
ros2 launch b2_base b2_base.launch.py network_interface:=enp3s0
```

For local topic testing without forcing an interface:

```bash
ros2 launch b2_base b2_base.launch.py
```

## Control

Publish velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

Useful services:

```bash
ros2 service call /b2_cmd_vel_bridge/stop_move std_srvs/srv/Trigger {}
ros2 service call /b2_cmd_vel_bridge/balance_stand std_srvs/srv/Trigger {}
ros2 service call /b2_cmd_vel_bridge/recovery_stand std_srvs/srv/Trigger {}
```

The launch subscribes `/sportmodestate` and `/lowstate`, publishes `/odom`, `/imu/data`, `/joint_states`, `/b2/status`, and sends `/api/sport/request` from `/cmd_vel`.
