# b2_driver_description

ROS 2 description package for the Unitree B2 robot.

Included assets:
- `urdf/` exported URDF
- `xacro/` upstream xacro sources copied from Unitree
- `meshes/` DAE assets
- `config/` control config and joint-name mapping
- `launch/check_joint.rviz` visualization config
- `usd/` Isaac-related asset

This workspace copy is ROS 2 only. Legacy ROS 1 launch files and old build
instructions were intentionally removed.

## Meshes

Two descriptions coexist and use different mesh sets:

- `urdf/b2_description.urdf` -- `base_link.dae` plus per-leg `FL_/FR_/RL_/RR_`
  hip, thigh and calf meshes.  This is what the launch files load.
- `xacro/robot.xacro` -- `trunk.dae` plus the generic `hip`, `thigh`,
  `thigh_mirror` and `calf` meshes.

Both sets are kept because both descriptions are live.  `dae/` used to hold a
byte-identical copy of `meshes/` and was removed, along with `trunk1.dae` and
`trunk2.dae`, which nothing referenced -- together 255 MB.

`trunk.dae` is still 46 MB on its own.  GitHub warns above 50 MB, so decimating
it is worth doing before it grows.
