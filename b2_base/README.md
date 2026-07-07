# b2_base

ROS 2 package for Unitree B2 state bridging, description bringup, and high-level
motion control.

## Build

```bash
cd /home/juno/ros2_ws
colcon build --packages-select b2_interface b2_driver_description b2_base
source install/setup.bash
```

## Launch

Use the Ethernet interface connected to the robot:

```bash
sudo /home/juno/ros2_ws/src/b2_driver/b2_base/scripts/set_unitree_static_ip.sh enp3s0
ros2 launch b2_base b2_bringup.launch.py network_interface:=enp3s0
```

For local topic testing without forcing an interface:

```bash
ros2 launch b2_base b2_bringup.launch.py
```

The static IP helper follows the Unitree ROS 2 docs default and sets the PC-side
address to `192.168.123.99/24` unless you override it.

## Control

Publish velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

Or publish the driver-specific motion command:

```bash
ros2 topic pub /b2/motion_command b2_interface/msg/B2MotionCommand "{command: move, linear_x: 0.2, linear_y: 0.0, angular_z: 0.0}"
```

Useful services:

```bash
ros2 service call /b2_driver_node/stop_move std_srvs/srv/Trigger {}
ros2 service call /b2_driver_node/balance_stand std_srvs/srv/Trigger {}
ros2 service call /b2_driver_node/recovery_stand std_srvs/srv/Trigger {}
```

The bringup subscribes `/sportmodestate` and `/lowstate`, publishes
`/b2/odom`, `/b2/imu`, `/b2/joint_states`, `/b2/foot_force`,
`/b2/battery_state`, `/b2/status`, `/b2/sport_state`, and sends
`/api/sport/request` from `/cmd_vel` or `/b2/motion_command`.
