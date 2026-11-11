# b2_base

ROS 2 package for Unitree B2 hardware bringup. The default hardware path is
the official Unitree SDK2; the former ROS/DDS bridge remains available only as
a compatibility transport.

SDK callback ownership, local stress measurements and reproduction commands are
documented in [Execution and performance](docs/performance.md).

`/b2/odom.twist` expresses Unitree's odometry-frame linear velocity in the physical
body frame named by `child_frame_id`, using the IMU quaternion (including roll/pitch).
Invalid quaternion or velocity data remains invalid instead of becoming a false stop.
`/b2/battery_state` uses BMS current in ROS amperes: negative on discharge, positive
on charge, with `present` and `power_supply_status` supplied. Verify the BMS current
sign on the target B2 before commissioning charging. The software command deadband
is per axis; the physical docking profile's 0.02 m/s planar floor accommodates the
default 0.01 deadband. A changed driver deadband needs a matching docking policy.

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

For SDK telemetry without any sport-control commands:

```bash
ros2 launch b2_base b2_bringup.launch.py network_interface:=enp3s0 enable_control:=false
```

The launch argument reaches the SDK driver as a read-only boolean parameter.
Velocity and motion/configuration/stop service requests are rejected with
`CONTROL_DISABLED`, and automatic startup standing is suppressed even if its
parameter is enabled. Telemetry, SDK no-op/version handshakes and read-only
controller queries continue. No lease or motion API is requested.
`/b2/driver_status.motion_ready` is false with reason `CONTROL_DISABLED`.
Start a control-enabled instance when this driver should own motion commands.

The static IP helper follows the Unitree ROS 2 docs default and sets the PC-side
address to `192.168.123.99/24` unless you override it.

## Control

Forward velocity is limited to `1.0 m/s`; reverse remains `0.8 m/s`.
`config/b2_driver_params.yaml` sets `max_linear_x` and `max_reverse_linear_x`
for both SDK2 and ROS/DDS transports. HMI and navigation may apply lower limits.

Publish velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

SDK2 motion-mode service and independent stop:

```bash
ros2 service call /b2/motion/set_mode b2_interface/srv/SetMotionMode "{mode: 1}"  # BALANCE_STAND
ros2 service call /b2/motion/set_mode b2_interface/srv/SetMotionMode "{mode: 3}"  # STAND_DOWN
ros2 service call /b2/motion/stop std_srvs/srv/Trigger {}
```

`SetMotionMode` also provides `DAMP`, `STAND_UP`, `RECOVERY_STAND`, free/classic/
vision walk, and low/high speed selection. See `b2_interface/srv/SetMotionMode`.

`STAND_DOWN` first requires an acknowledged stop and three fresh stationary
sport packets. `DAMP` remains available without fresh state. Both latch velocity
off: even a zero `cmd_vel` cannot issue `Move` after a rest request. A successful
explicit standing/walking preparation command is required to resume velocity;
a speed-level change does not resume it. Driver readiness reports
`POSTURE_MOTION_DISABLED` while this latch is held. A failed rest request retains
the latch, and an explicit stop remains available. SDK acknowledgement alone
does not establish that the physical posture transition has finished.

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
| out | `/b2/sport_status` | `b2_interface/SportStatus` |
| out | `/b2/motor_status` | `b2_interface/MotorStatusArray` |
| out | `/b2/driver_status` | `b2_interface/DriverStatus` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

## Hardware status and readiness

These additions apply to `transport:=sdk`. Sport status includes the SDK's raw
`error_code`, mode, gait type, body height and foot raise height. Body height is
reported separately from odometry Z; neither is the complete height of the robot
and its payload. Motor status lists the 12 joints in the same named order as
`/b2/joint_states`, with temperature, raw mode and raw loss counter. A cumulative
loss counter is not decoded as a current motor fault. Temperature trip limits are
not guessed from another Unitree model.

```bash
ros2 topic echo /b2/driver_status
ros2 topic echo /b2/sport_status
ros2 topic echo /b2/motor_status
ros2 topic echo /diagnostics
```

`/b2/comm_ok` retains its existing meaning: recent sport telemetry. The separate
`motion_ready` field additionally requires recent low-level telemetry, zero SDK
sport error code, and a successful recent `MotionSwitcher.CheckMode` reporting a
configured controller name and robot form. Its `reason` identifies the failed
condition. A failed condition rejects velocity commands, invalidates the cached
command and stops an active velocity stream. Recovery needs a **new** velocity
command; old commands are not replayed. This is driver readiness, not confirmation
that a requested posture has physically completed or an alternative to the
robot's safety manager.

The default accepted B2 legged form is `"0"`, with controller names `normal`, `ai`
and `advanced`, following the pinned SDK example. Read-only `ServiceList` and
`GetPkgVersion` results are also reported. Service status/protection integers are
exposed verbatim rather than assigned undocumented readiness meanings. Missing
optional service/version metadata does not block a successful `CheckMode`.

SDK polling has separate clients and a callback group with bounded timeouts.
It never selects, releases or switches the vendor controller. Explicit stop and
existing posture/recovery services remain callable when readiness is unavailable.
`controller.require_ready:=false` is an explicit compatibility override for
firmware without `CheckMode`; telemetry freshness and sport-error checks remain.
The status report records that override. Use a deployment parameter file to change
these read-only parameters; do not change them during a motion request.

## Body height, body attitude and gait settings

| Service | Type | SDK operation |
| --- | --- | --- |
| `/b2/motion/set_body_height` | `b2_interface/srv/SetBodyHeight` | `BodyHeight(height)` |
| `/b2/motion/set_body_pose` | `b2_interface/srv/SetBodyPose` | `Euler(roll, pitch, yaw)` |
| `/b2/motion/configure_gait` | `b2_interface/srv/ConfigureGait` | `SwitchGait`, `ContinuousGait`, `FastWalk`, `AutoRecoverySet` |
| `/b2/motion/configure_height_mode` | `b2_interface/srv/ConfigureHeightMode` | `FreeHeight`, `GaitHeight` |

The services are installed but their optional settings are disabled in the default
configuration. No height, body attitude, gait or automatic-recovery setting is
sent at startup. Each request checks its configured limits and current readiness,
invalidates cached velocity, and requires a successful `StopMove` plus three new
stationary sport packets before calling the setting API. Defaults are at most
0.03 m/s linear and 0.05 rad/s yaw speed, within a 2-second wait; change the
`configuration.*` deployment parameters when needed. Only one mode/setting operation
may own the SDK configuration client. Overlapping requests return
`MOTION_OPERATION_BUSY` immediately instead of waiting in a callback queue and
executing after a stop. The
independent stop path and telemetry remain responsive during a long setting RPC.
Invalid or disabled requests are rejected without an SDK call. Responses preserve
the vendor result code; `success` means SDK acknowledgement, not measured physical
completion. New velocity commands are required after a setting request.
An explicit stop during an in-flight setting makes its response unsuccessful
(`CONFIGURATION_INTERRUPTED_BY_STOP`) even if the later SDK acknowledgement is
zero. The original SDK code is retained and physical outcome is reported unknown.
Loss of readiness during a setting also requests stop even when no velocity is
cached. A later zero SDK response then returns `CONFIGURATION_ABORTED` with the
readiness failure, not success. Recovery/standing services still have their
existing semantics; an overlapping request is rejected as busy.

For height, set `body_height.enabled`, `body_height.minimum` and
`body_height.maximum` in your deployment copy of `b2_driver_params.yaml`. Verify
the **argument's units, absolute/relative reference, permitted modes and limits**
for the installed B2 firmware first. The public pinned header declares
`BodyHeight(float)` but does not specify these limits. The default zero limits are
placeholders, **not** a physical operating range. The advertised 645 mm standing
and 330 mm lying overall sizes do not specify low-height walking capability.

For attitude, set `body_pose.enabled` and the three `max_*_rad` limits. Angles are
radians and describe the body, not a Nav2 heading target. A zero axis limit permits
only zero on that axis. At least one positive limit is required when enabled.

For gait settings, set `gait_configuration.enabled`; `SWITCH_GAIT` additionally
requires the ID in `gait_configuration.allowed_ids`. No Go2 gait IDs are assumed.
`ConfigureGait.feature` selects `SWITCH_GAIT=0`, `CONTINUOUS_GAIT=1`, `FAST_WALK=2`
or `AUTO_RECOVERY=3`; `gait_id` applies only to the first, and `enabled` to the
others.

Height mode switches use `ConfigureHeightMode.feature`: `FREE_HEIGHT=0` or
`GAIT_HEIGHT=1`, with the requested boolean in `enabled`. Enable the startup
parameter `height_mode_configuration.enabled` only after verifying these raw SDK
switches on the installed firmware. These names do not specify a physical height
or establish how `BodyHeight` behaves. Both true and false requests use the same
stop/stationary/readiness checks as other configuration requests. Nothing is sent
at startup, and invalid features are rejected before any StopMove or setting RPC.

Inspect the installed service definition with:

```bash
ros2 interface show b2_interface/srv/ConfigureHeightMode
```

The SDK coverage review uses the pinned B2
[SportClient header](https://github.com/unitreerobotics/unitree_sdk2/blob/9754cd153af3da471b0fe5f3aa535e426fb11db3/include/unitree/robot/b2/sport/sport_client.hpp).

| SDK capability | Driver integration |
| --- | --- |
| Velocity, stop, standing/recovery, walk modes, speed presets | Existing `/cmd_vel`, stop and mode services |
| Body height/attitude, gait options, height mode switches | Typed configuration services with deployment gates |
| State, errors, motor feedback, controller and firmware inventory | Typed topics and diagnostics |
| `MoveToPos`, `TrajectoryFollow`, `SwitchMoveMode` | Direct SDK position/trajectory control is outside this Nav2 velocity driver |
| `HandStand` | Outside the inspection robot's driving and stair operations |

Changing body height also changes sensor ground clearance and total collision
height. These driver services do not silently change Nav2 height filters or fixed
sensor mounting transforms; deployment integration must use measured geometry.

## Verification without a robot

```bash
colcon build --packages-select b2_interface b2_base b2_motion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select b2_base b2_motion --return-code-on-test-failure \
  --ctest-args --output-on-failure
colcon test-result --test-result-base build/b2_base/test_results --verbose
```

The SDK test uses loopback-only fake sport, motion-switcher and robot-state peers.
Native state publication and SDK replies run independently of the ROS executor,
as they do on the external robot.
It checks telemetry mapping, raw diagnostics, firmware metadata, readiness and
fault gating, nonfinite/out-of-range rejection, configuration payloads and SDK
errors, plus stop responsiveness during delayed RPCs. Test height and angle bounds
are synthetic fixtures and are not deployment recommendations.
The same fixture runs with both `rmw_cyclonedds_cpp` and the hardware launch's
`rmw_fastrtps_cpp`. Runtime test XML records `busy_rejection_ms`,
`stop_during_configuration_ms` and `fault_to_stop_request_ms`; these are host
loopback measurements, not robot braking times. Regression cases also cover
overlapping requests, StopMove rejection, readiness loss during a setting and
unavailable optional firmware/service metadata.
Height-mode cases check both boolean values, wire API IDs 1052/1053, unsupported
features, default/read-only rejection, concurrent-operation rejection and raw
firmware errors. A mapping regression changes all joint, motor, cell-voltage and
foot-force inputs over 64 updates and checks the values and timestamps while
verifying that all seven vector storage addresses remain unchanged.

Static analysis covers production sources and tests separately, with cppcheck's
Google Test model for test macros. Both checks execute even on cppcheck versions
that ament normally skips for performance reasons. The source-header test checks
the existing WeGo proprietary SPDX and copyright policy; it does not relicense
vendor code. Formatting, Python lint, CMake lint and package XML checks also run.
The final 2026-11-09 run passed all 13 `b2_base` CTest targets and the
`b2_motion` posture runtime target, with no static-analysis skips. A separate
Fast DDS loopback launch confirmed `enable_control:=false`, `CONTROL_DISABLED`
status and rejection of a runtime speed-limit change with the original value
retained. All SDK peers and launch probes used loopback, without robot actuation.
The SDK fixture contains eight cases per middleware, including the reusable
low-state converter. The installed height-mode service also rejected both
features in the read-only launch, and its enable parameter rejected runtime
mutation with the original false value retained.

## Control-path review and optimization

| Problem | Implemented behavior | Regression check |
| --- | --- | --- |
| Queued setting could execute after an operator stop | Reject overlapping mode/setting requests immediately with `MOTION_OPERATION_BUSY` | Delayed setting, concurrent requests and explicit stop |
| Health loss during a setting could still return success | Invalidate the operation, request stop and preserve the SDK result with an unsuccessful response | Sport fault during a delayed setting |
| SDK launch ignored `enable_control` | Read-only mode rejects all motion and stop requests while retaining telemetry and probes | Read-only SDK fixture and loopback launch |
| Invalid startup numbers could reach clamping or timer code | Validate finite limits, signs and representable timer/RPC durations before operation | Invalid motion parameter fixture |
| Runtime parameter changes appeared successful without taking effect | Declare startup-only driver parameters read-only | Descriptor assertions and rejected launch-time parameter mutation |
| Each readiness check copied service/module metadata | Share one readiness decision routine; copy the full report only for status publication | Equivalent full/lightweight decisions and benchmark |

The command timer, watchdog, request admission and stationary wait use a small
readiness value with no metadata allocation. Status publication retains the full
message. Low-state conversion lives in `low_state_messages.hpp`: the SDK driver
allocates its joint, motor, battery-cell and foot-force buffers once, fills them
in place, then publishes through the existing const-reference ROS API. Fixed
joint names are initialized once. The default mutually-exclusive low-state timer
owns these reusable buffers; native SDK callbacks continue to copy incoming state
under their separate mutex. This removes conversion-side vector growth and joint
name construction, without claiming to remove middleware serialization or copies.
Diagnostics reserve their fixed entries and format copied motor values outside
the state mutex. SDK timeout validation also rejects nonfinite/out-of-float-range
values before narrowing. Command rates,
freshness limits, stop conditions and SDK wire payloads are unchanged.

Release-build measurements on the development host, 2026-11-09:

| ROS middleware | Full status snapshot | Lightweight readiness | Relative speed |
| --- | ---: | ---: | ---: |
| Cyclone DDS | 4.98 µs/call | 0.0387 µs/call | 129× |
| Fast DDS | 4.87 µs/call | 0.0386 µs/call | 126× |

Each synthetic report contains 64 services and 64 module versions. Values are the
median of seven alternating 10,000-call batches after warmup. The full status API
is the same-build baseline, and a checksum verifies matching decisions. These
numbers describe readiness lookup cost, not whole-driver CPU reduction, Nav2
throughput or physical stopping time. Runtime XML retains the measured values.
Physical height/attitude limits and firmware behavior still require verification
on the actual B2 before enabling those optional settings.

`transport:=ros_bridge` restores the old `/api/sport/request`, `/sportmodestate`
and `/lowstate` transport for diagnostics. Its battery output forwards every new LowState, as
the SDK transport does; it no longer throttles control feedback to a 1 Hz gauge
rate. Neither transport retimestamps cached telemetry after incoming data stops.

Do not run the compatibility bridge beside
the SDK driver: both would control the same B2 DDS endpoints.

The default TF edge remains `odom -> base_link` for the existing Nav2 and
MuJoCo stack. Set `odom_frame`, `base_frame`, and `imu_frame` together when a
multi-robot frame prefix is needed.

## Source measurement time

The SDK transport uses `SportModeState.stamp` by default (`source_time.mode: source`). The same original timestamp is used for body status, IMU, odometry and TF. `/b2/time_reference` preserves the original-to-converted time pair, and `B2/source_time` diagnostics retain source, converted and receipt times.

Zero, repeated, backwards, stale or future source times withhold measurements and motion readiness. The host clock must already match the robot clock domain, or `source_time.offset_ns` must be configured from independently verified evidence; there is no automatic one-way delay calibration. A host clock step requires restarting after synchronization. `source_time.mode: receipt` is an explicit legacy comparison mode and is diagnosed as unsynchronized. LowState/joint timestamps and the ROS-only compatibility transport are not qualified by this change. Actual B2 clock semantics and drift still require physical verification.
