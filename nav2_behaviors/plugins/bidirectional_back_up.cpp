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

#include "nav2_behaviors/plugins/bidirectional_back_up.hpp"

namespace nav2_behaviors
{
void BidirectionalBackUp::onConfigure()
{
  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error{ "Failed to lock node" };
  }
  stepback_completed_pub = node->create_publisher<std_msgs::msg::Empty>("stepback_completed", 1);
  check_to_run_stepback_recovery_client = node->create_client<std_srvs::srv::Trigger>("check_to_run_stepback_recovery");
  DriveOnHeading::onConfigure();  // need to run parent onConfigure, else parameters won't be loaded
}

Status BidirectionalBackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  auto check_run_stepback_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto stepback_future = check_to_run_stepback_recovery_client->async_send_request(check_run_stepback_req);

  if (stepback_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
  {
    auto stepback_result = stepback_future.get();
    RCLCPP_INFO(logger_, "Got service result: %d", stepback_result->success);
    if (!stepback_result->success)
    {
      RCLCPP_INFO(logger_, "No need to run stepback recovery due to timout or mileage, returning success");
      return Status::SUCCEEDED;
    }
  }
  else
  {
    RCLCPP_ERROR(logger_, "Service call timed out");
    return Status::FAILED;
  }

  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;

  if (command->target.y != 0.0 || command->target.z != 0.0)
  {
    RCLCPP_INFO(logger_, "Backing up in Y and Z not supported, will only move in X.");
    return Status::FAILED;
  }

  RCLCPP_INFO(logger_, "Last navigating direction: %s", last_navigating_dir_.c_str());

  // Silently ensure that both the speed and direction are negative.
  if (last_navigating_dir_ == "forward")
  {
    RCLCPP_INFO(logger_, "Last navigating direction is forward, setting comand x and command speed to negative");
    command_x_ = -std::fabs(command->target.x);
    command_speed_ = -std::fabs(command->speed);
  }
  else if (last_navigating_dir_ == "backward")
  {
    RCLCPP_INFO(logger_, "Last navigating direction is backward, setting comand x and command speed to position");
    command_x_ = std::fabs(command->target.x);
    command_speed_ = std::fabs(command->speed);
  }
  else
  {
    RCLCPP_WARN(logger_, "Last navigating direction is not forward or backward, not running backup");
    return Status::SUCCEEDED;
  }

  if (!nav2_util::getCurrentPose(initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }
  RCLCPP_INFO(logger_, "Stepback recovery sent to drive_on_heading, publishing to stepback_completed topic");
  std_msgs::msg::Empty msg;
  stepback_completed_pub->publish(msg);
  return Status::SUCCEEDED;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BidirectionalBackUp, nav2_core::Behavior)
