// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_WITH_TIMEOUT_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_WITH_TIMEOUT_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_behavior_tree
{
/**
 * @brief The RecoveryNodeWithTimeout has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 *
 */
class RecoveryNodeWithTimeout : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RecoveryNodeWithTimeout
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RecoveryNodeWithTimeout(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for nav2_behavior_tree::RecoveryNodeWithTimeout
   */
  ~RecoveryNodeWithTimeout() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  unsigned int current_child_idx_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr check_pathblock_timer_expire_client;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pathblock_timer_control_publisher_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_WITH_TIMEOUT_HPP_
