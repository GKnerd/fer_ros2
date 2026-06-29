# Attribution and Third-Party Notices

`fer_moveit_config` is a downstream rework of the long-running
[`panda_moveit_config`](https://github.com/moveit/panda_moveit_config) package
(originally hosted under the `ros-planning` GitHub organization, now under
`moveit`). The configuration files (`config/*.yaml`, `rviz/*.rviz`) are
derived from that lineage and remain under the BSD 3-Clause License
declared in [`LICENSE`](LICENSE).

Acknowledgements to the upstream contributors whose work this package
builds on, including (in no particular order):

- Rick Staa
- Robert Haschke (Bielefeld University)
- Marco Boneberger (Franka Robotics)
- Matt Droter
- Thore Goll
- Tim Redick
- and the broader MoveIt community

## Third-party files with their own license

- `launch/fer_moveit_launch.py` is an adapted version of
  [`moveit_resources/panda_moveit_config/launch/demo.launch.py`](https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py),
  originally distributed under the **Apache License, Version 2.0**, with
  modifications by Franka Robotics GmbH and Georgios Katranis. The Apache
  2.0 header at the top of the file is preserved as required by that
  license. A copy of the Apache 2.0 license is available at
  <http://www.apache.org/licenses/LICENSE-2.0>.

If you believe an attribution is missing or incorrect, please open an
issue on the repository.
