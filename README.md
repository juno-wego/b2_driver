# b2_driver

ROS 2 driver for the Unitree B2. Hardware uses Unitree SDK2 directly; the
legacy Unitree ROS/DDS bridge is retained only for compatibility diagnostics.

- `b2_base` — SDK2 hardware node, legacy bridge nodes, and bringup launch files
- `b2_description` — meshes, URDF, RViz config, description launch

This repository covers the **physical robot** only. Simulation lives in its own
repository; see [Simulation](#simulation) below.

## Dependencies

The legacy bridge nodes speak Unitree's `unitree_go` and `unitree_api` message
packages. They are built directly from the pinned official
[`b2_interface/upstream/unitree_ros2/`](b2_interface/upstream/unitree_ros2/)
submodule. Initialize submodules after cloning:

```bash
git submodule update --init --recursive
```

The default hardware driver likewise uses the pinned official
[`b2_base/upstream/unitree_sdk2/`](b2_base/upstream/unitree_sdk2/) submodule.

The description package is installed as **`b2_driver_description`** so it does
not collide with the upstream Unitree package also named `b2_description`.

Install the ROS message-generation and DDS support once:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rmw-fastrtps-cpp \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

## Build

From your colcon workspace root, with this repository cloned under `src/`:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-up-to \
  unitree_go unitree_api b2_base b2_driver_description
source install/setup.bash
```

## Bring-up

Connect the robot over Ethernet, then set the PC-side address on that interface
and launch. Substitute your own interface name for `enp3s0`:

```bash
sudo "$(ros2 pkg prefix b2_base)/lib/b2_base/set_unitree_static_ip.sh" enp3s0
ros2 launch b2_base b2_bringup.launch.py network_interface:=enp3s0
```

This default `transport:=sdk` builds against the pinned Unitree SDK2 submodule.
The launch selects Fast DDS for the ROS graph so SDK2 can own its separate
CycloneDDS participant.

The helper follows the Unitree ROS 2 docs default and sets the PC to
`192.168.123.99/24` unless overridden.

For legacy ROS/DDS bridge diagnostics without binding an interface:

```bash
ros2 launch b2_base b2_bringup.launch.py transport:=ros_bridge
```

## Interface

The default SDK2 driver receives Unitree DDS state directly and publishes
`/b2/odom`, `/b2/imu/data`, `/b2/joint_states`, `/b2/battery_state`,
`/b2/foot_force`, `/b2/mode_code`, and `/b2/comm_ok`. It publishes TF
`odom -> base_link` by default. Velocity commands arrive on `/cmd_vel` and call
SDK2 `SportClient::Move()` directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

See [`b2_base/wego/README.md`](b2_base/wego/README.md) for the full topic, service and
message list.

## DDS configuration

`b2_base/wego/config/cyclonedds_mujoco.xml` and `cyclonedds_client.xml` pin a fixed
`<ParticipantIndex>` so a single native-DDS peer can be discovered on loopback
without colliding with a ROS 2 domain-0 session.

**A fixed participant index only works for one participant.** Do not export
these for a multi-node ROS 2 launch: every node after the first fails with
`rtps_init: failed to create unicast sockets ... participant index N`. Use them
only for the single-process native SDK tools.

They are only for `transport:=ros_bridge`; refer to them by prefix rather than by a path
into the source tree:

```bash
export CYCLONEDDS_URI="file://$(ros2 pkg prefix b2_base)/share/b2_base/config/cyclonedds_client.xml"
```

## Simulation

The MuJoCo simulation, the trained RL velocity policy, and the SLAM/Nav2
bring-up live in the separate **`b2_simulation`** repository.

The simulator owns its own low-level `unitree_go/LowState`/`LowCmd` policy path;
it does not start `b2_sdk_driver`. Both layers expose the shared upper-level
topics `/cmd_vel`, `/b2/imu/data`, `/b2/joint_states`, and
`/b2/battery_state`, while the simulator's ground-truth odometry remains on
`/b2/odom_gt`. Drive it with `b2_simulation`'s policy runner rather than either
hardware transport.
