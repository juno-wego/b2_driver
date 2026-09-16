# Provenance record — B2 driver

Reviewed: 2026-09-16

## Wego wrapper scope

`b2_base/wego/` and `b2_interface/wego/` contain the ROS 2 adapter layer:
command/state bridges, launch/configuration, and the `SetMotionMode` service.
The files use Unitree SDK2 headers and Unitree ROS message types, but no
copyright or SPDX notice belonging to Unitree is present in these wrapper
files.  The repository history records their separation from the Unitree
upstream trees in commit `c68a3ab`.

These wrapper directories are the only paths covered by this repository's
Wego Proprietary License.  A later source-origin review must be performed
before relicensing any individual file that is found to contain copied or
modified Unitree source; that file must retain the applicable BSD-3-Clause
copyright and license notice.

## Unitree upstream scope

The following pinned upstream trees retain their own BSD-3-Clause LICENSE
texts and must not be relicensed by Wego:

- `b2_base/upstream/unitree_sdk2/`
- `b2_interface/upstream/unitree_ros2/`

## B2 description assets — release blocker

The B2 description package contains meshes, URDF, Xacro, and USD assets. Its
initial addition is associated with the repository history entry
`6c11812` ("unitree 코드 가져오기 및 예제 코드 생성중"), and no asset license or
vendor redistribution permission is recorded in this repository.

Therefore `b2_description/` is intentionally excluded from the Wego
Proprietary License.  Do not include it in a customer artifact until one of
the following is recorded:

1. Unitree's license covering the exact source/asset distribution,
2. Unitree's written redistribution permission, or
3. replacement assets created or licensed by Wego.
