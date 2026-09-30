#include "fer_moveit_config/core/checks.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fer_moveit_config
{
bool speed_valid(double speed_scaling)
{
  return speed_scaling > 0.0 && speed_scaling <= 1.0;
}

bool at_rest(const std::vector<double> & velocities, double threshold)
{
  return std::all_of(
    velocities.begin(), velocities.end(),
    [threshold](double v) {return std::abs(v) < threshold;});
}

double progress(double elapsed, double duration)
{
  if (duration <= 0.0) {
    return 1.0;
  }
  return std::clamp(elapsed / duration, 0.0, 1.0);
}

}  // namespace fer_moveit_config
