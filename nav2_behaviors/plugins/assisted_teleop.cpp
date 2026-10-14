// Copyright (c) 2022 Joshua Wallace
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

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "nav2_behaviors/plugins/assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
AssistedTeleop::AssistedTeleop()
: TimedBehavior<AssistedTeleopAction>(),
  feedback_(std::make_shared<AssistedTeleopAction::Feedback>())
{}

void AssistedTeleop::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // set up parameters
  nav2_util::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulation_time_step", rclcpp::ParameterValue(0.1));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "cmd_vel_teleop", rclcpp::ParameterValue(std::string("cmd_vel_teleop")));

  // Empty keeps the behavior server's shared cmd_vel
  nav2_util::declare_parameter_if_not_declared(
    node,
    "cmd_vel_teleop_out", rclcpp::ParameterValue(std::string("")));

  // A teleop command older than this counts as zero, 0 = never
  nav2_util::declare_parameter_if_not_declared(
    node,
    "cmd_vel_teleop_timeout", rclcpp::ParameterValue(0.0));

  // Look at least this far ahead however slow the command, 0 = off
  nav2_util::declare_parameter_if_not_declared(
    node,
    "min_projection_distance", rclcpp::ParameterValue(0.0));

  // Stop on any predicted collision instead of slowing down
  nav2_util::declare_parameter_if_not_declared(
    node,
    "stop_on_collision", rclcpp::ParameterValue(false));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "local_footprint_topic",
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

  node->get_parameter("projection_time", projection_time_);
  node->get_parameter("simulation_time_step", simulation_time_step_);
  node->get_parameter("cmd_vel_teleop_timeout", cmd_vel_teleop_timeout_);
  node->get_parameter("min_projection_distance", min_projection_distance_);
  node->get_parameter("stop_on_collision", stop_on_collision_);

  std::string cmd_vel_teleop;
  node->get_parameter("cmd_vel_teleop", cmd_vel_teleop);

  // Replaces the base class publisher before activate() runs, so stopRobot() follows it
  std::string cmd_vel_teleop_out;
  node->get_parameter("cmd_vel_teleop_out", cmd_vel_teleop_out);
  if (!cmd_vel_teleop_out.empty()) {
    vel_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, cmd_vel_teleop_out, 1);
  }

  vel_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    node,
    cmd_vel_teleop, rclcpp::SystemDefaultsQoS(),
    [&](geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      teleop_twist_.twist = *msg;
      teleop_time_ = clock_->now();
    }, [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      teleop_twist_ = *msg;
      teleop_time_ = clock_->now();
    });

  preempt_teleop_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "preempt_teleop", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::preemptTeleopCallback,
      this, std::placeholders::_1));

  std::string footprint_topic;
  node->get_parameter("local_footprint_topic", footprint_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    node_, footprint_topic, *tf_, robot_base_frame_, transform_tolerance_);

  obstacle_pub_ = node->create_publisher<std_msgs::msg::Bool>("teleop_obstacle", 1);
}

void AssistedTeleop::activate()
{
  stopping_ = false;
  TimedBehavior<AssistedTeleopAction>::activate();
  obstacle_pub_->on_activate();
}

void AssistedTeleop::deactivate()
{
  stopping_ = true;
  obstacle_pub_->on_deactivate();
  TimedBehavior<AssistedTeleopAction>::deactivate();
}

ResultStatus AssistedTeleop::onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command)
{
  preempt_teleop_ = false;
  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;
  // Only commands sent after the goal starts count
  std::lock_guard<std::mutex> lock(teleop_mutex_);
  teleop_twist_ = geometry_msgs::msg::TwistStamped();
  return ResultStatus{Status::SUCCEEDED, AssistedTeleopActionResult::NONE};
}

void AssistedTeleop::onActionCompletion(std::shared_ptr<AssistedTeleopActionResult>/*result*/)
{
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    teleop_twist_ = geometry_msgs::msg::TwistStamped();
  }
  preempt_teleop_ = false;
  setObstacle(false);
}

ResultStatus AssistedTeleop::onCycleUpdate()
{
  // A goal with no time allowance never ends by itself, so deactivation would wait forever
  if (stopping_) {
    return ResultStatus{Status::FAILED, AssistedTeleopActionResult::UNKNOWN};
  }

  feedback_->current_teleop_duration = elasped_time_;
  action_server_->publish_feedback(feedback_);

  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN_STREAM(
      logger_,
      "Exceeded time allowance before reaching the " << behavior_name_.c_str() <<
        "goal - Exiting " << behavior_name_.c_str());
    return ResultStatus{Status::FAILED, AssistedTeleopActionResult::TIMEOUT};
  }

  // user states that teleop was successful
  if (preempt_teleop_) {
    stopRobot();
    return ResultStatus{Status::SUCCEEDED, AssistedTeleopActionResult::NONE};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Current robot pose is not available for " <<
        behavior_name_.c_str());
    return ResultStatus{Status::FAILED, AssistedTeleopActionResult::TF_ERROR};
  }

  geometry_msgs::msg::TwistStamped teleop_twist;
  rclcpp::Time teleop_time;
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    teleop_twist = teleop_twist_;
    teleop_time = teleop_time_;
  }
  if (cmd_vel_teleop_timeout_ > 0.0 &&
    (this->clock_->now() - teleop_time).seconds() > cmd_vel_teleop_timeout_)
  {
    teleop_twist.twist = geometry_msgs::msg::Twist();
  }
  const auto & twist = teleop_twist.twist;
  auto scaled_twist = std::make_unique<geometry_msgs::msg::TwistStamped>(teleop_twist);

  // Nothing commanded, nothing to collide with
  constexpr double kZero = 1e-3;
  if (std::abs(twist.linear.x) < kZero && std::abs(twist.linear.y) < kZero &&
    std::abs(twist.angular.z) < kZero)
  {
    scaled_twist->twist = geometry_msgs::msg::Twist();
    setObstacle(false);
    vel_pub_->publish(std::move(scaled_twist));
    return ResultStatus{Status::RUNNING, AssistedTeleopActionResult::NONE};
  }

  // Slow commands still look min_projection_distance ahead, counting the footprint's swing
  double horizon = projection_time_;
  if (min_projection_distance_ > 0.0) {
    const double speed = std::hypot(twist.linear.x, twist.linear.y) +
      std::abs(twist.angular.z) * footprintRadius();
    if (speed > 1e-6) {
      horizon = std::max(horizon, min_projection_distance_ / speed);
    }
  }
  constexpr double kMaxSteps = 500.0;
  const int steps = static_cast<int>(
    std::clamp(std::ceil(horizon / simulation_time_step_), 1.0, kMaxSteps));
  const double step_time = horizon / steps;

  geometry_msgs::msg::Pose2D projected_pose;
  projected_pose.x = current_pose.pose.position.x;
  projected_pose.y = current_pose.pose.position.y;
  projected_pose.theta = tf2::getYaw(current_pose.pose.orientation);

  bool collided = false;
  for (int i = 1; i <= steps; ++i) {
    projected_pose = projectPose(projected_pose, twist, step_time);

    // Costmap and footprint are fetched once per cycle
    if (!local_collision_checker_->isCollisionFree(projected_pose, i == 1)) {
      const double time = i * step_time;
      collided = true;
      if (stop_on_collision_ || i == 1) {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          *clock_,
          1000,
          behavior_name_.c_str() << " collision predicted in " << time <<
            " seconds, setting velocity to zero");
        scaled_twist->twist.linear.x = 0.0f;
        scaled_twist->twist.linear.y = 0.0f;
        scaled_twist->twist.angular.z = 0.0f;
      } else {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          *clock_,
          1000,
          behavior_name_.c_str() << " collision approaching in " << time << " seconds");
        double scale_factor = time / horizon;
        scaled_twist->twist.linear.x *= scale_factor;
        scaled_twist->twist.linear.y *= scale_factor;
        scaled_twist->twist.angular.z *= scale_factor;
      }
      break;
    }
  }
  setObstacle(collided);
  vel_pub_->publish(std::move(scaled_twist));

  return ResultStatus{Status::RUNNING, AssistedTeleopActionResult::NONE};
}

geometry_msgs::msg::Pose2D AssistedTeleop::projectPose(
  const geometry_msgs::msg::Pose2D & pose,
  const geometry_msgs::msg::Twist & twist,
  double projection_time)
{
  geometry_msgs::msg::Pose2D projected_pose = pose;

  projected_pose.x += projection_time * (
    twist.linear.x * cos(pose.theta) -
    twist.linear.y * sin(pose.theta));

  projected_pose.y += projection_time * (
    twist.linear.x * sin(pose.theta) +
    twist.linear.y * cos(pose.theta));

  projected_pose.theta += projection_time * twist.angular.z;

  return projected_pose;
}

void AssistedTeleop::preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_teleop_ = true;
}

void AssistedTeleop::setObstacle(bool obstacle)
{
  if (obstacle == obstacle_) {
    return;
  }
  obstacle_ = obstacle;
  std_msgs::msg::Bool msg;
  msg.data = obstacle;
  obstacle_pub_->publish(msg);
}

double AssistedTeleop::footprintRadius()
{
  // The footprint is fixed, so it is looked up only until it first resolves
  if (footprint_radius_ > 0.0) {
    return footprint_radius_;
  }
  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header header;
  if (footprint_sub_->getFootprintInRobotFrame(footprint, header)) {
    for (const auto & point : footprint) {
      footprint_radius_ = std::max(footprint_radius_, std::hypot(point.x, point.y));
    }
  }
  return footprint_radius_;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::AssistedTeleop, nav2_core::Behavior)
