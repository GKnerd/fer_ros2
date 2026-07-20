## 2026-07-20

### franka_semantic_components
- `src/franka_robot_state.cpp` — `get_robot_state_ptr()`, `get_values_as_message()`:
  replaced deprecated `LoanedStateInterface::get_value()` with `get_optional()`.
  Why: `get_value()` collapses a failed read into quiet NaN; this code `bit_cast`s
  that value into a `franka::RobotState*`, so a failed read became a wild pointer
  dereference. `get_optional()` lets us detect the failure before casting.
  Also converted the interface-existence check to an early return.
- `src/franka_robot_model.cpp` - `update_state_and_model()` also has the same changes in
  `src/franka_robot_model.cpp:66-67`.

### franka_hardware
- Removed stale `#include <hardware_interface/visibility_control.h>` from
  `include/franka_hardware/real/franka_multi_hardware_interface.hpp`.

### franka_robot_state_broadcaster
- **Behavioral change:** default `arm_id` changed from `"panda"` to `"fer"` in both
  broadcasters (`on_init()`, `auto_declare<std::string>`) and in
  `config/franka_robot_state_broadcaster_parameters.yaml` (`default_value`).
- Moved `franka_robot_state_broadcaster_parameters.yaml` from `src/` to `config/`, and
  updated the `generate_parameter_library()` path in `CMakeLists.txt` to match.
  The build breaks if only one of the two is changed.
- Updated deprecated header from `#include <realtime_tools/realtime_publisher.h>` to
  `#include <realtime_tools/realtime_publisher.hpp>` (`.h` form removed in Jazzy),
  in both `franka_robot_state_broadcaster.hpp` and `franka_robot_model_broadcaster.hpp`.
- Removed deprecated header `#include <rclcpp/qos_event.hpp>` from
  `src/franka_robot_state_broadcaster.cpp` and `src/franka_robot_model_broadcaster.cpp`.

