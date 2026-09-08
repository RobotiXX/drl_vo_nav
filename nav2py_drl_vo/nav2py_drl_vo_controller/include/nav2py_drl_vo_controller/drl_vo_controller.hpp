/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NAV2PY_DRL_VO_CONTROLLER__DRL_VO_CONTROLLER_HPP_
#define NAV2PY_DRL_VO_CONTROLLER__DRL_VO_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2py/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace nav2py_drl_vo_controller
{

class DrlVoController : public nav2py::Controller
{
public:
  DrlVoController() = default;
  ~DrlVoController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void pedestrianMapCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr message);
  void sendInputs(
    const std::vector<float> & current_scan,
    const std::vector<float> & scan_history,
    const std::vector<float> & pedestrian_map,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    bool scan_changed,
    bool pedestrian_map_changed);
  bool commandIsCollisionFree(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & command) const;
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    double lookahead_distance,
    const nav_msgs::msg::Path & transformed_plan) const;
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second,
    double radius) const;
  bool transformPose(
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("DrlVoController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
  global_plan_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    pedestrian_map_subscription_;

  mutable std::mutex sensor_mutex_;
  mutable std::mutex plan_mutex_;
  nav_msgs::msg::Path global_plan_;
  std::deque<std::vector<float>> scan_history_;
  std::vector<float> current_scan_;
  std::vector<float> pedestrian_map_;
  rclcpp::Time last_scan_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pedestrian_map_received_{0, 0, RCL_ROS_TIME};
  std::uint64_t scan_generation_{0U};
  std::uint64_t pedestrian_map_generation_{0U};
  std::uint64_t sent_scan_generation_{0U};
  std::uint64_t sent_pedestrian_map_generation_{0U};

  double lookahead_distance_{2.0};
  double transform_tolerance_seconds_{0.1};
  double scan_timeout_seconds_{0.5};
  double pedestrian_timeout_seconds_{0.5};
  double command_timeout_seconds_{0.2};
  double range_limit_{30.0};
  double model_scan_angle_min_{-2.09439510239};
  double model_scan_angle_max_{2.09439510239};
  double max_linear_speed_{0.5};
  double max_angular_speed_{0.7};
  double active_linear_speed_limit_{0.5};
  double min_obstacle_distance_{0.45};
  double collision_check_horizon_{1.0};
  double collision_check_step_{0.05};
  bool require_pedestrian_data_{true};
};

}  // namespace nav2py_drl_vo_controller

#endif  // NAV2PY_DRL_VO_CONTROLLER__DRL_VO_CONTROLLER_HPP_
