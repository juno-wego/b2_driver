# SDK driver execution and performance

This report covers the SDK hardware path in `b2_sdk_driver`, measured on the
development host on 2026-11-12. The SDK peer is a local fake on DDS domain 155 and
interface `lo`. These results measure host scheduling and message preparation;
they do not measure B2 gait response, Ethernet latency, physical stopping or
marker docking accuracy.

## Ownership and execution

| Component | Responsibility | Execution |
| --- | --- | --- |
| `B2SdkDriver` | ROS interfaces, bounded command cache, stop generation and mode ownership | Main ROS executor: 2 workers |
| `SdkCallbackExecutors` | Command RPC timer and mode/configuration/stop services | Command executor: 1 worker; operations executor: 2 workers |
| `SportStateMessages` | Reused IMU, odometry, TF and mode messages | Main state timer only |
| `LowStateMessages` | Reused joints, motor, force and battery messages | Main low-state timer only |
| `DriverStatus` | State validity, readiness and read-only controller information | Locked snapshots; SDK metadata polling has a separate callback group |
| Native SDK subscribers | Receive and validate source state, update snapshots | SDK-owned receive threads |

The original main executor used four workers. The current layout uses five ROS
executor workers in total: two main workers plus three for SDK calls. SDK receive
threads are additional in both versions. This is isolation of blocking calls,
not a reduction in thread count or a claim of reduced whole-process CPU use.

Command, mode and stop callback groups are manually attached to their executors.
The stop group can run while the mode group waits for a reply. SDK clients keep
their bounded timeouts; no detached jobs or growing command-work queue is used.
The existing motion mutex serializes velocity and explicit StopMove RPCs. A mode
operation mutex immediately rejects overlapping configuration/mode requests.
Old velocity commands are discarded across mode changes and stops.

The owned executors use the node's ROS context. Destruction waits for workers
to enter or finish `spin()` before cancelling, then joins both executors before
SDK clients and callback targets are destroyed. A test immediately constructs
and destroys the executors twenty times with an independent context.

The 20 Hz command timer, 20 ms watchdog, 33 ms sport-state timer and 100 ms
low-state timer retain their configured behavior. Speed/deadband policy is
independent of this optimization; no minimum physical walking speed is inferred.

## Delayed SDK response

`SdkRpcProgress.DelayedModeKeepsTelemetryAndStopResponsive` deliberately holds a
mode RPC for 1.5 seconds. An observer on its own ROS executor subscribes to
odometry and requests StopMove while that mode RPC is pending. The caller's
main executor is separately exercised with one, two and four workers.

Before the change, the one-worker case failed: no odometry samples arrived in
the observation window, the maximum odometry gap was 1544.87 ms, and the stop
reply did not arrive within 300 ms. The 300 ms number is a capped wait, not a
measurement of the eventual stop round-trip time. The original four-worker
configuration already passed: 33.18 ms maximum odometry gap and 4.97 ms stop
reply in that run.

The baseline XML did not capture the ROS RMW name; the CSV labels it
`not_recorded`. Use that failure as a scheduling reproduction, not a precise
comparison of latency between RMW implementations.

The final CycloneDDS and Fast DDS runs, including the new two-worker production
configuration, are recorded in [rpc_progress.csv](data/rpc_progress.csv). These
are individual runs under local host scheduling, not latency guarantees.

| Final main executor | ROS RMW | Maximum odometry gap | Stop reply during mode RPC |
| --- | --- | ---: | ---: |
| 2 workers | CycloneDDS | 33.26 ms | 8.84 ms |
| 2 workers | Fast DDS (hardware launch default) | 33.14 ms | 8.30 ms |

## Message preparation

`SportStateMessages` initializes frame names and covariance storage once, then
updates only dynamic fields. The existing velocity conversion, quaternion,
source timestamp and topic contents are preserved. SDK receive callbacks never
mutate these output messages. The main timer publishes through the ROS
`publish(const Message &)` overload before the next update.

`benchmark_state_messages` compares fresh message construction with reuse in
the same Release executable. It alternates order, warms up twenty batches,
then measures 200 batches of 1000 updates. The CSV p50/p95 values are percentiles
of **batch mean nanoseconds per update**, not individual ROS callback latency.
DDS, serialization and publication are excluded. Three complete runs are
retained in [state_messages.csv](data/state_messages.csv).

| Frame names | Fresh construction | Reused storage | Saving per update |
| --- | ---: | ---: | ---: |
| Default short names | 68.004 ns | 11.049 ns | 56.955 ns |
| Namespaced names | 118.546 ns | 11.188 ns | 107.358 ns |

Each table value is the median of the three runs' p50 batch means. The raw CSV
also retains p95 values, including host scheduling outliers. Build details are
in [environment.json](data/environment.json).

The absolute saving is small: message preparation is only part of the driver.
No whole-node speedup, CPU reduction or increased docking accuracy is inferred
from this microbenchmark.

## Rest posture and command ordering

The previous implementation accepted new `cmd_vel` after an acknowledged
`StandDown`, including `Move(0)`. The regression fixture observed seven extra
Move calls in 400 ms. These calls can conflict with the requested rest posture.

`StandDown` and `Damp` now latch velocity off before the request. Cached and
new velocity commands, including zeros, cannot emit Move. An acknowledged
explicit standing/walking preparation command clears this latch; a speed-level
change does not. `/b2/driver_status` reports `motion_ready=false` with
`POSTURE_MOTION_DISABLED` while the latch is held.

Before `StandDown`, the driver requires an acknowledged StopMove and three new
valid stationary sport packets. It reuses `configuration.stop_confirm_timeout_s`
and the stationary thresholds in the deployment configuration. A newer stop
interrupts this preparation. Clearing the owned pending stop is serialized with
the command timer so an ordinary delayed stop cannot follow the rest request.
An explicit operator stop remains available. `Damp` remains callable when
state is stale or StopMove fails. Failed rest requests retain the velocity latch.

The regression test checks zero additional Move/ordinary StopMove calls after
StandDown, recovery after StandUp, rejection without measured stop or StopMove
acknowledgement, and availability of Damp without fresh feedback. An SDK
acknowledgement still does not establish physical posture completion. Docking's
marker-sit workflow separately checks fresh posture mode, stable body height
and stopped odometry after the response.

## Reproduction

From the colcon workspace, after a Release build:

```bash
source install/setup.bash
/usr/bin/ctest --test-dir build/b2_base \
  -R '^(test_sdk_safety|test_sdk_safety_fastrtps|test_state_bridge|velocity_limits)$' \
  --output-on-failure
build/b2_base/benchmark_state_messages 200
```

The SDK tests use the loopback fake and separate test domains configured by
CMake. `test_sdk_safety` and `test_sdk_safety_fastrtps` run serially. Run the
standalone microbenchmark without concurrent compilation for comparable data.
Physical B2 commissioning must still establish source-clock alignment, firmware
posture codes, camera calibration, minimum walking command and final accuracy.

The final local run passed eight CTest groups, including all sixteen SDK cases
on each ROS RMW, source timestamp checks, state conversion, velocity limits,
proprietary source headers and both cppcheck groups. The related marker-sit
workflow passed all nine runtime/configuration cases. None contacts a B2.
