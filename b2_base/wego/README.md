# b2_base

ROS 2 package for Unitree B2 hardware bringup. The default hardware path is
the official Unitree SDK2; the former ROS/DDS bridge remains available only as
a compatibility transport.

## Build

From your colcon workspace root:

```bash
colcon build --packages-up-to b2_base
source install/setup.bash
```

## Launch

Use the Ethernet interface connected to the robot:

```bash
sudo "$(ros2 pkg prefix b2_base)/lib/b2_base/set_unitree_static_ip.sh" enp3s0
ros2 launch b2_base b2_bringup.launch.py network_interface:=enp3s0
```

For legacy ROS/DDS bridge diagnostics without forcing an interface:

```bash
ros2 launch b2_base b2_bringup.launch.py transport:=ros_bridge
```

The static IP helper follows the Unitree ROS 2 docs default and sets the PC-side
address to `192.168.123.99/24` unless you override it.

## Control

Publish velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

SDK2 motion-mode service and independent emergency stop:

```bash
ros2 service call /b2/motion/set_mode b2_interface/srv/SetMotionMode "{mode: 1}"  # BALANCE_STAND
ros2 service call /b2/motion/set_mode b2_interface/srv/SetMotionMode "{mode: 3}"  # STAND_DOWN
ros2 service call /b2/motion/stop std_srvs/srv/Trigger {}
```

`SetMotionMode` also provides `DAMP`, `STAND_UP`, `RECOVERY_STAND`, free/classic/
vision walk, and low/high speed selection. See `b2_interface/srv/SetMotionMode`.

The default SDK2 driver receives native DDS state directly and exposes the ROS
contract below. It keeps `/cmd_vel` as the default input for the existing Nav2
configuration; set `cmd_vel_topic:=/b2/cmd_vel` when the stack is namespaced.
It launches rclcpp with `rmw_fastrtps_cpp`: SDK2 owns a separate CycloneDDS
participant, and using `rmw_cyclonedds_cpp` for both in one process conflicts.

| Direction | Topic | Type |
| --- | --- | --- |
| in | `/cmd_vel` | `geometry_msgs/Twist` |
| out | `/b2/odom` | `nav_msgs/Odometry` |
| out | `/b2/imu/data` | `sensor_msgs/Imu` |
| out | `/b2/joint_states` | `sensor_msgs/JointState` |
| out | `/b2/battery_state` | `sensor_msgs/BatteryState` |
| out | `/b2/foot_force`, `/b2/mode_code`, `/b2/comm_ok` | standard ROS messages |

`transport:=ros_bridge` restores the old `/api/sport/request` and
`/sportmodestate`/`/lowstate` transport for diagnostics. Do not run it beside
the SDK driver: both would control the same B2 DDS endpoints.

The default TF edge remains `odom -> base_link` for the existing Nav2 and
MuJoCo stack. Set `odom_frame`, `base_frame`, and `imu_frame` together when a
multi-robot frame prefix is needed.
