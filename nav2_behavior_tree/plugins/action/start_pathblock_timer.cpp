// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/start_pathblock_timer.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

StartPathblockTimer::StartPathblockTimer(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  pathblock_timer_control_pub_ = node_->create_publisher<std_msgs::msg::String>("pathblock_timer_control", 1);
}

BT::NodeStatus StartPathblockTimer::tick()
{
  std_msgs::msg::String data;
  data.data = "start";
  pathblock_timer_control_pub_->publish(data);
  return BT::NodeStatus::SUCCESS;  // always return failure to keep recovery tree running
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::StartPathblockTimer>("StartPathblockTimer");
}
