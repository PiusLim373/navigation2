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

#include <string>
#include "nav2_behavior_tree/plugins/control/recovery_node_with_timeout.hpp"

namespace nav2_behavior_tree
{

RecoveryNodeWithTimeout::RecoveryNodeWithTimeout(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  check_pathblock_timer_expire_client = node_->create_client<std_srvs::srv::Trigger>("check_pathblock_timer_expire");
  pathblock_timer_control_publisher_ = node_->create_publisher<std_msgs::msg::String>("pathblock_timer_control", 1);
}

BT::NodeStatus RecoveryNodeWithTimeout::tick()
{
  callback_group_executor_.spin_some();
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }
  setStatus(BT::NodeStatus::RUNNING);
  while (current_child_idx_ < children_count) {
    auto future = check_pathblock_timer_expire_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "Failed to call service check_to_run_stepback_recovery" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    auto result = future.get();
    if (result->success)
    {
      std::cout << "Pathblock controller returning path block timer has expired, returning FAILURE" << std::endl; 
      return BT::NodeStatus::FAILURE;
    }
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // reset node and return success when first child returns success
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        case BT::NodeStatus::FAILURE:
          {
            // halt first child and tick second child in next iteration
            ControlNode::haltChild(0);
            current_child_idx_++;
            break;
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // halt second child, increment recovery count, and tick first child in next iteration
            ControlNode::haltChild(1);
            current_child_idx_--;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            // reset node and return failure if second child fails
            halt();
            return BT::NodeStatus::FAILURE;
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNodeWithTimeout::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  std_msgs::msg::String msg;
  msg.data = "reset";
  pathblock_timer_control_publisher_->publish(msg);

}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNodeWithTimeout>("RecoveryNodeWithTimeout");
}
