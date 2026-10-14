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

#ifndef NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/twist_subscriber.hpp"

namespace nav2_behaviors
{
using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;

/**
 * @class nav2_behaviors::AssistedTeleop
 * @brief An action server behavior for assisted teleop
 */
class AssistedTeleop : public TimedBehavior<AssistedTeleopAction>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using AssistedTeleopActionGoal = AssistedTeleopAction::Goal;
  using AssistedTeleopActionResult = AssistedTeleopAction::Result;
  AssistedTeleop();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const AssistedTeleopActionGoal> command) override;

  /**
   * @brief func to run at the completion of the action
   */
  void onActionCompletion(std::shared_ptr<AssistedTeleopActionResult>/*result*/) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

  void activate() override;
  void deactivate() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief project a position
   * @param pose initial pose to project
   * @param twist velocity to project pose by
   * @param projection_time time to project by
   */
  geometry_msgs::msg::Pose2D projectPose(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Twist & twist,
    double projection_time);

  /**
   * @brief Callback function to preempt assisted teleop
   * @param msg empty message
   */
  void preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr msg);

  /**
   * @brief Publish teleop_obstacle when the value changes
   */
  void setObstacle(bool obstacle);

  /**
   * @brief Distance of the farthest footprint vertex, 0 until a footprint arrives
   */
  double footprintRadius();

  AssistedTeleopAction::Feedback::SharedPtr feedback_;

  // parameters
  double projection_time_;
  double simulation_time_step_;
  double cmd_vel_teleop_timeout_{0.0};
  double min_projection_distance_{0.0};
  bool stop_on_collision_{false};

  // Written by the subscription, read by the action thread
  std::mutex teleop_mutex_;
  geometry_msgs::msg::TwistStamped teleop_twist_;
  rclcpp::Time teleop_time_{0, 0, RCL_ROS_TIME};
  bool preempt_teleop_{false};
  bool obstacle_{false};
  double footprint_radius_{0.0};
  // Set before the action server deactivates, which waits for the running goal to end
  std::atomic<bool> stopping_{false};

  // subscribers
  std::unique_ptr<nav2_util::TwistSubscriber> vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_teleop_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  // publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
