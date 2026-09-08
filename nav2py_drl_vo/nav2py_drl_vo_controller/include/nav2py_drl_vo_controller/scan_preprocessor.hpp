/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NAV2PY_DRL_VO_CONTROLLER__SCAN_PREPROCESSOR_HPP_
#define NAV2PY_DRL_VO_CONTROLLER__SCAN_PREPROCESSOR_HPP_

#include <cstddef>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2py_drl_vo_controller
{

std::vector<float> resample_scan(
  const sensor_msgs::msg::LaserScan & scan,
  std::size_t output_samples,
  double output_angle_min,
  double output_angle_max);

}  // namespace nav2py_drl_vo_controller

#endif  // NAV2PY_DRL_VO_CONTROLLER__SCAN_PREPROCESSOR_HPP_
