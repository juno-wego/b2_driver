# b2_driver

ROS 2 workspace for the Unitree B2 driver, organized to mirror the cleaner
`go2_driver` split:

- `b2_base`: state/control bridge nodes and bringup launch files
- `b2_description`: robot meshes, URDF, RViz config, and description launch
- `b2_interface`: custom ROS 2 messages used by the bridge nodes

The shared `unitree_go` and `unitree_api` ROS 2 interface packages are used
from the local vendored copy in `/home/juno/ros2_ws/src/unitree_ros2_vendor`,
not directly from `/home/juno/ros2_ws/src/unitree`.

Note: the description package is published as `b2_driver_description` to avoid
conflicting with the upstream Unitree package named `b2_description` that
already exists elsewhere in this workspace.
