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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__START_PATHBLOCK_TIMER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__START_PATHBLOCK_TIMER_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The StartPathblockTimer behavior is used to switch the controller
 * that will be used by the controller server. It subscribes to a topic "PATH_BLOCK_TIMER"
 * to get the decision about what controller must be used. It is usually used before of
 * the FollowPath. The selected_controller output port is passed to controller_id
 * input port of the FollowPath
 */
class StartPathblockTimer : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::StartPathblockTimer
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  StartPathblockTimer(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the PATH_BLOCK_TIMER topic
   *
   * @param msg the message with the id of the PATH_BLOCK_TIMER
   */

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pathblock_timer_control_pub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__START_PATHBLOCK_TIMER_HPP_
