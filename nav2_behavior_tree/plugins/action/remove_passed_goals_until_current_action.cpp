// Copyright (c) 2021 Samsung Research America
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

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_until_current_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoalsUntilCurrent::RemovePassedGoalsUntilCurrent(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5)
{
  getInput("radius", viapoint_achieved_radius_);

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus RemovePassedGoalsUntilCurrent::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  // std::cout << "goal poses size: " << goal_poses.size() << std::endl;
  double closest_distance = std::numeric_limits<double>::max();
  unsigned int closest_goal_index = 0;
  for (unsigned int i = 0; i < goal_poses.size(); ++i) {
    double distance = euclidean_distance(goal_poses[i].pose, current_pose.pose);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_goal_index = i;
    }
  }

  // return the goals that are after the closest goal waypoint
  Goals new_goal_poses;
  for (unsigned int i = closest_goal_index; i < goal_poses.size(); ++i) {
    new_goal_poses.push_back(goal_poses[i]);
  }
  // std::cout << "closest goal index: " << closest_goal_index << std::endl;
  // std::cout << "current pose: "
  //           << current_pose.pose.position.x << ", "
  //           << current_pose.pose.position.y << std::endl;
  // std::cout << "closest goal pose: "
  //           << goal_poses[closest_goal_index].pose.position.x << ", "
  //           << goal_poses[closest_goal_index].pose.position.y << std::endl;
  // std::cout << "new goal poses size: " << new_goal_poses.size() << std::endl;

  setOutput("output_goals", new_goal_poses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoalsUntilCurrent>("RemovePassedGoalsUntilCurrent");
}
