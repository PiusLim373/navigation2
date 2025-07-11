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

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/plugins/action/compute_interpolation_path_through_poses_action.hpp"

namespace nav2_behavior_tree
{

ComputeInterpolationPathThroughPosesAction::ComputeInterpolationPathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputeInterpolationPathThroughPosesAction::on_tick()
{
  getInput("goals", goal_.goals);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
  getInput("check_only", check_only);
}

BT::NodeStatus ComputeInterpolationPathThroughPosesAction::on_success()
{
  nav_msgs::msg::Path empty_path;
  if (check_only == "true")
  {
    std::cout << "Check only mode, writing to temp path" << std::endl;
    setOutput("temp_path", result_.result->path);
    setOutput("path", empty_path);
  }
  else
  {
    std::cout << "actual generation, writing to path" << std::endl;
    setOutput("temp_path", empty_path);
    setOutput("path", result_.result->path);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeInterpolationPathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  setOutput("temp_path", empty_path);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeInterpolationPathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  setOutput("temp_path", empty_path);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeInterpolationPathThroughPosesAction>(
        name, "compute_interpolation_path_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeInterpolationPathThroughPosesAction>(
    "ComputeInterpolationPathThroughPoses", builder);
}
