/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "nav2py_drl_vo_controller/scan_preprocessor.hpp"

namespace
{

sensor_msgs::msg::LaserScan make_scan(std::size_t sample_count)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / static_cast<double>(sample_count - 1U);
  scan.angle_max = scan.angle_min + scan.angle_increment *
    static_cast<double>(sample_count - 1U);
  scan.ranges.resize(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    scan.ranges[index] = static_cast<float>(index + 1U);
  }
  return scan;
}

TEST(ScanPreprocessor, ResamplesConfiguredFieldOfView)
{
  const auto scan = make_scan(721U);
  const auto result = nav2py_drl_vo_controller::resample_scan(
    scan, 5U, -M_PI_2, M_PI_2);
  ASSERT_EQ(result.size(), 5U);
  EXPECT_FLOAT_EQ(result.front(), scan.ranges[180]);
  EXPECT_FLOAT_EQ(result[2], scan.ranges[360]);
  EXPECT_FLOAT_EQ(result.back(), scan.ranges[540]);
}

TEST(ScanPreprocessor, InvalidRangesBecomeTrainingCompatibleZeros)
{
  auto scan = make_scan(721U);
  scan.ranges[360] = std::numeric_limits<float>::infinity();
  const auto result = nav2py_drl_vo_controller::resample_scan(
    scan, 3U, -M_PI_2, M_PI_2);
  EXPECT_FLOAT_EQ(result[1], 0.0F);
}

TEST(ScanPreprocessor, RejectsInsufficientAngularCoverage)
{
  auto scan = make_scan(181U);
  scan.angle_min = -M_PI_4;
  scan.angle_increment = M_PI_2 / 180.0;
  EXPECT_THROW(
    nav2py_drl_vo_controller::resample_scan(scan, 720U, -2.0, 2.0),
    std::invalid_argument);
}

}  // namespace
