// Copyright (c) 2023 Dexory
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include "angles/angles.h"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

void PoseProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  SimpleProgressChecker::initialize(parent, plugin_name);
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_angle", rclcpp::ParameterValue(0.5));
  node->get_parameter_or(plugin_name + ".required_movement_angle", required_movement_angle_, 0.5);

  // Empty keeps the upstream behaviour
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".pause_status_topic", rclcpp::ParameterValue(std::string("")));
  std::string pause_status_topic;
  node->get_parameter(plugin_name + ".pause_status_topic", pause_status_topic);
  if (!pause_status_topic.empty()) {
    pause_sub_ = node->create_subscription<tmms_msgs::msg::QuadrupedMainStatus>(
      pause_status_topic, 10,
      [this](const tmms_msgs::msg::QuadrupedMainStatus::SharedPtr msg) {
        paused_ = msg->is_paused;
        pause_status_ns_ = clock_->now().nanoseconds();
      });
  }

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PoseProgressChecker::dynamicParametersCallback, this, _1));
}

bool PoseProgressChecker::isPaused() const
{
  // The status arrives at 5 Hz; a silent publisher must not hold the check open forever
  constexpr int64_t kStaleNs = 1000000000;
  return paused_ && clock_->now().nanoseconds() - pause_status_ns_ < kStaleNs;
}

bool PoseProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // Held still on purpose: restart the time allowance from when the pause ends
  if (isPaused()) {
    reset();
    return true;
  }

  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if (!baseline_pose_set_ || PoseProgressChecker::isRobotMovedEnough(current_pose2d)) {
    resetBaselinePose(current_pose2d);
    return true;
  }
  return clock_->now() - baseline_time_ <= time_allowance_;
}

bool PoseProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_ ||
         poseAngleDistance(pose, baseline_pose_) > required_movement_angle_;
}

double PoseProgressChecker::poseAngleDistance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  return abs(angles::shortest_angular_distance(pose1.theta, pose2.theta));
}

rcl_interfaces::msg::SetParametersResult
PoseProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_angle") {
        required_movement_angle_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::PoseProgressChecker, nav2_core::ProgressChecker)
