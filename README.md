This is a gazebo ros repo for a linefollowing robot.

## Vendoring PX4 optical flow plugin

If you vendor PX4 optical flow sources into this repo, place them at:

`third_party/px4_optical_flow/optical_flow`

Expected upstream source path:
`PX4-Autopilot/src/modules/simulation/gz_plugins/optical_flow`

This package will include the vendored plugin by calling that folder's own
`CMakeLists.txt` (if present), instead of guessing Gazebo library versions in
this repo's top-level CMake.

Build and run:

```bash
colcon build --packages-select linebot
source install/setup.bash
ros2 launch linebot sim.launch.py
```
