# Third-party notices — B2 driver

This repository combines Wego-authored ROS 2 wrapper code with the following
separately licensed Unitree components.

| Component | Location | License | Customer artifact action |
| --- | --- | --- | --- |
| Unitree SDK2 | `b2_base/upstream/unitree_sdk2/` | BSD-3-Clause | Include its LICENSE text and copyright notice. |
| Unitree ROS2 | `b2_interface/upstream/unitree_ros2/` | BSD-3-Clause | Include its LICENSE text and copyright notice. |
| B2 description meshes, URDF, USD | `b2_description/` | Provenance pending | Do not redistribute until Unitree's applicable asset license or written permission is recorded. |

The Wego proprietary license at the repository root applies only to
`b2_base/wego/` and `b2_interface/wego/`; it does not alter any third-party
license.

For a customer runtime, copy the two Unitree license texts into the product's
`LICENSES/` directory and retain this notice.  This document is a delivery
record, not a substitute for vendor permission covering the B2 description
assets.
