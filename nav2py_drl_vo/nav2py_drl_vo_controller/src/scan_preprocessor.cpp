/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nav2py_drl_vo_controller/scan_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav2py_drl_vo_controller
{

std::vector<float> resample_scan(
  const sensor_msgs::msg::LaserScan & scan,
  std::size_t output_samples,
  double output_angle_min,
  double output_angle_max)
{
  if (scan.ranges.empty()) {
    throw std::invalid_argument("LaserScan has no ranges");
  }
  if (!std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0) {
    throw std::invalid_argument("LaserScan angle_increment must be positive");
  }
  if (output_samples < 2U || output_angle_max <= output_angle_min) {
    throw std::invalid_argument("invalid DRL-VO output scan geometry");
  }

  const double input_angle_max = scan.angle_min +
    scan.angle_increment * static_cast<double>(scan.ranges.size() - 1U);
  constexpr double angle_epsilon = 1.0e-5;
  if (output_angle_min < scan.angle_min - angle_epsilon ||
    output_angle_max > input_angle_max + angle_epsilon)
  {
    throw std::invalid_argument("LaserScan does not cover the configured DRL-VO field of view");
  }

  std::vector<float> result(output_samples, 0.0F);
  const double output_increment = (output_angle_max - output_angle_min) /
    static_cast<double>(output_samples - 1U);
  for (std::size_t output_index = 0; output_index < output_samples; ++output_index) {
    const double angle = output_angle_min + output_increment * static_cast<double>(output_index);
    const double floating_index = (angle - scan.angle_min) / scan.angle_increment;
    const auto input_index = static_cast<std::size_t>(std::clamp(
        std::llround(floating_index), 0LL,
        static_cast<long long>(scan.ranges.size() - 1U)));
    const float value = scan.ranges[input_index];
    if (std::isfinite(value) && value > 0.0F) {
      result[output_index] = value;
    }
  }
  return result;
}

}  // namespace nav2py_drl_vo_controller
