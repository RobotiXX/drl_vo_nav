/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nav2py_drl_vo_controller/drl_vo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2py_drl_vo_controller/scan_preprocessor.hpp"

namespace nav2py_drl_vo_controller
{
namespace
{

constexpr std::size_t kScanHistoryFrames = 10U;
constexpr std::size_t kModelScanSamples = 720U;
constexpr std::size_t kPedestrianMapSize = 2U * 80U * 80U;

template<typename Iterator, typename Getter>
Iterator min_by(Iterator begin, Iterator end, Getter getter)
{
  if (begin == end) {
    return end;
  }
  auto lowest_value = getter(*begin);
  auto lowest = begin;
  for (auto current = std::next(begin); current != end; ++current) {
    const auto value = getter(*current);
    if (value < lowest_value) {
      lowest_value = value;
      lowest = current;
    }
  }
  return lowest;
}

std::string shell_quote(const std::string & value)
{
  std::string quoted{"'"};
  for (const char character : value) {
    if (character == '\'') {
      quoted += "'\\''";
    } else {
      quoted += character;
    }
  }
  quoted += '\'';
  return quoted;
}

std::string base64_encode(const std::vector<uint8_t> & bytes)
{
  static constexpr char alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(4U * ((bytes.size() + 2U) / 3U));
  for (std::size_t index = 0; index < bytes.size(); index += 3U) {
    const uint32_t first = bytes[index];
    const uint32_t second = index + 1U < bytes.size() ? bytes[index + 1U] : 0U;
    const uint32_t third = index + 2U < bytes.size() ? bytes[index + 2U] : 0U;
    const uint32_t value = (first << 16U) | (second << 8U) | third;
    output.push_back(alphabet[(value >> 18U) & 0x3fU]);
    output.push_back(alphabet[(value >> 12U) & 0x3fU]);
    output.push_back(index + 1U < bytes.size() ? alphabet[(value >> 6U) & 0x3fU] : '=');
    output.push_back(index + 2U < bytes.size() ? alphabet[value & 0x3fU] : '=');
  }
  return output;
}

std::string numeric_array_payload(const std::vector<float> & values)
{
  // Use an explicit big-endian IEEE-754 representation so the Python decoder
  // does not depend on CPU byte order.
  std::vector<uint8_t> bytes;
  bytes.reserve(values.size() * sizeof(float));
  for (const float value : values) {
    uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    bytes.push_back(static_cast<uint8_t>((bits >> 24U) & 0xffU));
    bytes.push_back(static_cast<uint8_t>((bits >> 16U) & 0xffU));
    bytes.push_back(static_cast<uint8_t>((bits >> 8U) & 0xffU));
    bytes.push_back(static_cast<uint8_t>(bits & 0xffU));
  }
  return "b64f32be:" + base64_encode(bytes);
}

}  // namespace

void DrlVoController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  const auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("DRL-VO controller node expired during configure");
  }
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const auto declare = [&node, this](const std::string & parameter, const auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + parameter, rclcpp::ParameterValue(value));
    };
  declare("lookahead_dist", 2.0);
  declare("transform_tolerance", 0.1);
  declare("scan_topic", std::string("/scan"));
  declare("scan_timeout", 0.5);
  declare("pedestrian_map_topic", std::string("/drl_vo/pedestrian_velocity_map"));
  declare("pedestrian_timeout", 0.5);
  declare("require_pedestrian_data", true);
  declare("command_timeout", 0.2);
  declare("model_file", std::string());
  declare("model_device", std::string("auto"));
  declare("range_limit", 30.0);
  declare("model_scan_angle_min", -2.09439510239);
  declare("model_scan_angle_max", 2.09439510239);
  declare("max_linear_speed", 0.5);
  declare("max_angular_speed", 0.7);
  declare("min_obstacle_distance", 0.45);
  declare("collision_check_horizon", 1.0);
  declare("collision_check_step", 0.05);

  std::string scan_topic;
  std::string pedestrian_map_topic;
  std::string model_file;
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_seconds_);
  node->get_parameter(plugin_name_ + ".scan_topic", scan_topic);
  node->get_parameter(plugin_name_ + ".scan_timeout", scan_timeout_seconds_);
  node->get_parameter(plugin_name_ + ".pedestrian_map_topic", pedestrian_map_topic);
  node->get_parameter(plugin_name_ + ".pedestrian_timeout", pedestrian_timeout_seconds_);
  node->get_parameter(plugin_name_ + ".require_pedestrian_data", require_pedestrian_data_);
  node->get_parameter(plugin_name_ + ".command_timeout", command_timeout_seconds_);
  node->get_parameter(plugin_name_ + ".model_file", model_file);
  node->get_parameter(plugin_name_ + ".model_device", model_device_);
  node->get_parameter(plugin_name_ + ".range_limit", range_limit_);
  node->get_parameter(plugin_name_ + ".model_scan_angle_min", model_scan_angle_min_);
  node->get_parameter(plugin_name_ + ".model_scan_angle_max", model_scan_angle_max_);
  node->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(plugin_name_ + ".min_obstacle_distance", min_obstacle_distance_);
  node->get_parameter(plugin_name_ + ".collision_check_horizon", collision_check_horizon_);
  node->get_parameter(plugin_name_ + ".collision_check_step", collision_check_step_);

  if (lookahead_distance_ <= 0.0 || scan_timeout_seconds_ <= 0.0 ||
    pedestrian_timeout_seconds_ <= 0.0 || command_timeout_seconds_ <= 0.0 ||
    range_limit_ <= 0.0 || max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
    collision_check_horizon_ <= 0.0 || collision_check_step_ <= 0.0)
  {
    throw nav2_core::PlannerException("DRL-VO controller parameters must be positive");
  }
  if (model_device_.empty()) {
    throw nav2_core::PlannerException("DRL-VO model_device cannot be empty");
  }
  active_linear_speed_limit_ = max_linear_speed_;
  pedestrian_map_.assign(kPedestrianMapSize, 0.0F);

  if (model_file.empty()) {
    model_file = ament_index_cpp::get_package_share_directory(
      "nav2py_drl_vo_controller") + "/model/drl_vo.zip";
  }
  const std::string executable = ament_index_cpp::get_package_prefix(
    "nav2py_drl_vo_controller") + "/lib/nav2py_drl_vo_controller/nav2py_run";
  std::ostringstream command;
  command << shell_quote(executable) << " --host 127.0.0.1 --port 0"
          << " --model-file " << shell_quote(model_file)
          << " --model-device " << shell_quote(model_device_)
          << " --range-limit " << range_limit_
          << " --vx-limit " << max_linear_speed_
          << " --wz-limit " << max_angular_speed_
          << " --min-obstacle-distance " << min_obstacle_distance_;
  nav2py_bootstrap(command.str());

  global_plan_publisher_ = node->create_publisher<nav_msgs::msg::Path>(
    "received_global_plan", 1);
  scan_subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&DrlVoController::scanCallback, this, std::placeholders::_1));
  pedestrian_map_subscription_ =
    node->create_subscription<std_msgs::msg::Float32MultiArray>(
    pedestrian_map_topic, rclcpp::SensorDataQoS(),
    std::bind(&DrlVoController::pedestrianMapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "Configured %s with checkpoint %s", plugin_name_.c_str(),
    model_file.c_str());
  if (!require_pedestrian_data_) {
    RCLCPP_WARN(
      logger_,
      "Pedestrian data is optional: this is a lidar-only DRL-VO ablation, not the social policy");
  }
}

void DrlVoController::cleanup()
{
  nav2py_cleanup();
  global_plan_publisher_.reset();
  scan_subscription_.reset();
  pedestrian_map_subscription_.reset();
  std::scoped_lock lock(sensor_mutex_, plan_mutex_);
  current_scan_.clear();
  scan_history_.clear();
  global_plan_.poses.clear();
  scan_generation_ = 0U;
  pedestrian_map_generation_ = 0U;
  sent_scan_generation_ = 0U;
  sent_pedestrian_map_generation_ = 0U;
}

void DrlVoController::activate()
{
  global_plan_publisher_->on_activate();
}

void DrlVoController::deactivate()
{
  global_plan_publisher_->on_deactivate();
}

void DrlVoController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    active_linear_speed_limit_ = max_linear_speed_;
  } else if (percentage) {
    active_linear_speed_limit_ = max_linear_speed_ *
      std::clamp(speed_limit, 0.0, 100.0) / 100.0;
  } else {
    active_linear_speed_limit_ = std::min(speed_limit, max_linear_speed_);
  }
}

void DrlVoController::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  try {
    auto processed = resample_scan(
      *message, kModelScanSamples, model_scan_angle_min_, model_scan_angle_max_);
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    current_scan_ = processed;
    scan_history_.push_back(std::move(processed));
    while (scan_history_.size() > kScanHistoryFrames) {
      scan_history_.pop_front();
    }
    last_scan_received_ = clock_->now();
    ++scan_generation_;
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000, "Rejected /scan for DRL-VO: %s", error.what());
  }
}

void DrlVoController::pedestrianMapCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr message)
{
  if (message->data.size() != kPedestrianMapSize ||
    !std::all_of(
      message->data.begin(), message->data.end(),
      [](float value) {return std::isfinite(value);}))
  {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "Rejected pedestrian velocity map: expected 12800 finite values");
    return;
  }
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  pedestrian_map_ = message->data;
  last_pedestrian_map_received_ = clock_->now();
  ++pedestrian_map_generation_;
}

geometry_msgs::msg::TwistStamped DrlVoController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  const auto transformed_plan = transformGlobalPlan(pose);
  const auto goal_pose = getLookAheadPoint(lookahead_distance_, transformed_plan);

  std::vector<float> current_scan;
  std::vector<float> scan_history;
  std::vector<float> pedestrian_map;
  rclcpp::Time scan_received(0, 0, clock_->get_clock_type());
  rclcpp::Time pedestrians_received(0, 0, clock_->get_clock_type());
  std::uint64_t scan_generation = 0U;
  std::uint64_t pedestrian_map_generation = 0U;
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (scan_history_.size() != kScanHistoryFrames) {
      throw nav2_core::PlannerException("DRL-VO scan history is not ready");
    }
    current_scan = current_scan_;
    scan_history.reserve(kScanHistoryFrames * kModelScanSamples);
    for (const auto & scan : scan_history_) {
      scan_history.insert(scan_history.end(), scan.begin(), scan.end());
    }
    pedestrian_map = pedestrian_map_;
    scan_received = last_scan_received_;
    pedestrians_received = last_pedestrian_map_received_;
    scan_generation = scan_generation_;
    pedestrian_map_generation = pedestrian_map_generation_;
  }

  const auto now = clock_->now();
  if ((now - scan_received).seconds() > scan_timeout_seconds_) {
    throw nav2_core::PlannerException("DRL-VO scan is stale");
  }
  if (require_pedestrian_data_ &&
    (pedestrians_received.nanoseconds() == 0 ||
    (now - pedestrians_received).seconds() > pedestrian_timeout_seconds_))
  {
    throw nav2_core::PlannerException("DRL-VO pedestrian data is unavailable or stale");
  }

  const bool scan_changed = scan_generation != sent_scan_generation_;
  const bool pedestrian_map_changed =
    pedestrian_map_generation != sent_pedestrian_map_generation_;
  sendInputs(
    current_scan, scan_history, pedestrian_map, goal_pose,
    scan_changed, pedestrian_map_changed);
  if (scan_changed) {
    sent_scan_generation_ = scan_generation;
  }
  if (pedestrian_map_changed) {
    sent_pedestrian_map_generation_ = pedestrian_map_generation;
  }
  geometry_msgs::msg::Twist command;
  try {
    command = wait_for_cmd_vel(
      std::chrono::milliseconds(
        static_cast<int64_t>(command_timeout_seconds_ * 1000.0)));
  } catch (const std::exception & error) {
    throw nav2_core::PlannerException(
            std::string("DRL-VO inference process failed: ") + error.what());
  }
  if (!std::isfinite(command.linear.x) || !std::isfinite(command.angular.z)) {
    throw nav2_core::PlannerException("DRL-VO returned a non-finite command");
  }
  command.linear.x = std::clamp(command.linear.x, 0.0, active_linear_speed_limit_);
  command.angular.z = std::clamp(
    command.angular.z, -max_angular_speed_, max_angular_speed_);
  if (!commandIsCollisionFree(pose, command)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "DRL-VO command rejected by local costmap collision check");
    command = geometry_msgs::msg::Twist();
  }

  geometry_msgs::msg::TwistStamped result;
  result.header.frame_id = pose.header.frame_id;
  result.header.stamp = now;
  result.twist = command;
  return result;
}

void DrlVoController::sendInputs(
  const std::vector<float> & current_scan,
  const std::vector<float> & scan_history,
  const std::vector<float> & pedestrian_map,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  bool scan_changed,
  bool pedestrian_map_changed)
{
  // YAML formatting and parsing dominated the controller budget for these
  // large fixed-size arrays. Base64 keeps nav2py's delimiter framing intact,
  // while NumPy decodes the endian-defined float storage without a YAML tree.
  if (scan_changed) {
    nav2py_send("scan", {numeric_array_payload(current_scan)});
    nav2py_send("scan_history", {numeric_array_payload(scan_history)});
  }
  if (pedestrian_map_changed) {
    nav2py_send("pedestrian_map", {numeric_array_payload(pedestrian_map)});
  }

  std::ostringstream goal;
  goal << "goal_pose:\n  position:\n    x: " << goal_pose.pose.position.x
       << "\n    y: " << goal_pose.pose.position.y << "\n";
  nav2py_send("data", {goal.str()});
}

bool DrlVoController::commandIsCollisionFree(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & command) const
{
  auto * costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  checker(costmap);
  const auto footprint = costmap_ros_->getRobotFootprint();
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  const int steps = static_cast<int>(std::ceil(
      collision_check_horizon_ / collision_check_step_));
  for (int step = 0; step <= steps; ++step) {
    const double cost = checker.footprintCostAtPose(x, y, yaw, footprint);
    if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return false;
    }
    x += command.linear.x * std::cos(yaw) * collision_check_step_;
    y += command.linear.x * std::sin(yaw) * collision_check_step_;
    yaw += command.angular.z * collision_check_step_;
  }
  return true;
}

void DrlVoController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
  global_plan_publisher_->publish(path);
}

geometry_msgs::msg::Point DrlVoController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second,
  double radius) const
{
  const double delta_x = second.x - first.x;
  const double delta_y = second.y - first.y;
  const double squared_length = delta_x * delta_x + delta_y * delta_y;
  if (squared_length <= std::numeric_limits<double>::epsilon()) {
    return second;
  }
  const double determinant = first.x * second.y - second.x * first.y;
  const double first_distance = first.x * first.x + first.y * first.y;
  const double second_distance = second.x * second.x + second.y * second.y;
  const double discriminant = std::max(
    0.0, radius * radius * squared_length - determinant * determinant);
  const double root = std::sqrt(discriminant);
  const double direction = std::copysign(1.0, second_distance - first_distance);
  geometry_msgs::msg::Point result;
  result.x = (determinant * delta_y + direction * delta_x * root) / squared_length;
  result.y = (-determinant * delta_x + direction * delta_y * root) / squared_length;
  return result;
}

geometry_msgs::msg::PoseStamped DrlVoController::getLookAheadPoint(
  double lookahead_distance,
  const nav_msgs::msg::Path & transformed_plan) const
{
  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("DRL-VO transformed plan is empty");
  }
  if (transformed_plan.poses.size() == 1U) {
    return transformed_plan.poses.front();
  }

  auto goal = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [lookahead_distance](const auto & pose) {
      return std::hypot(pose.pose.position.x, pose.pose.position.y) >=
      lookahead_distance;
    });
  if (goal == transformed_plan.poses.end()) {
    goal = std::prev(transformed_plan.poses.end());
    const auto previous = std::prev(goal);
    const double heading = std::atan2(
      goal->pose.position.y - previous->pose.position.y,
      goal->pose.position.x - previous->pose.position.x);
    auto projected = goal->pose.position;
    projected.x += std::cos(heading) * lookahead_distance;
    projected.y += std::sin(heading) * lookahead_distance;
    auto output = *goal;
    output.pose.position = circleSegmentIntersection(
      goal->pose.position, projected, lookahead_distance);
    return output;
  }
  if (goal != transformed_plan.poses.begin()) {
    auto output = *goal;
    output.pose.position = circleSegmentIntersection(
      std::prev(goal)->pose.position, goal->pose.position, lookahead_distance);
    return output;
  }
  return *goal;
}

nav_msgs::msg::Path DrlVoController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("DRL-VO received a zero-length plan");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into plan frame");
  }
  auto * costmap = costmap_ros_->getCostmap();
  const double distance_threshold = std::max(
    costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;
  const auto begin = min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const auto & plan_pose) {
      return nav2_util::geometry_utils::euclidean_distance(robot_pose, plan_pose);
    });
  const auto end = std::find_if(
    begin, global_plan_.poses.end(),
    [&robot_pose, distance_threshold](const auto & plan_pose) {
      return nav2_util::geometry_utils::euclidean_distance(robot_pose, plan_pose) >
      distance_threshold;
    });

  nav_msgs::msg::Path transformed;
  transformed.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed.header.stamp = pose.header.stamp;
  for (auto current = begin; current != end; ++current) {
    geometry_msgs::msg::PoseStamped stamped = *current;
    stamped.header.frame_id = global_plan_.header.frame_id;
    stamped.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped local;
    if (!transformPose(costmap_ros_->getBaseFrameID(), stamped, local)) {
      throw nav2_core::PlannerException("Unable to transform global plan pose");
    }
    transformed.poses.push_back(local);
  }
  global_plan_.poses.erase(global_plan_.poses.begin(), begin);
  global_plan_publisher_->publish(transformed);
  if (transformed.poses.empty()) {
    throw nav2_core::PlannerException("DRL-VO transformed plan has no poses");
  }
  return transformed;
}

bool DrlVoController::transformPose(
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output) const
{
  if (input.header.frame_id == frame) {
    output = input;
    return true;
  }
  try {
    output = tf_->transform(
      input, frame, tf2::durationFromSec(transform_tolerance_seconds_));
    return true;
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000, "DRL-VO transform failed: %s", error.what());
    return false;
  }
}

}  // namespace nav2py_drl_vo_controller

PLUGINLIB_EXPORT_CLASS(
  nav2py_drl_vo_controller::DrlVoController, nav2_core::Controller)
