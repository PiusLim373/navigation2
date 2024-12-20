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
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0),
  timeout_(15.0)
{
  getInput("number_of_retries", number_of_retries_);
  getInput("timeout", timeout_);
}

BT::NodeStatus RecoveryNodeWithTimeout::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }
  if (start_time_ == std::chrono::steady_clock::time_point{}) {
    start_time_ = std::chrono::steady_clock::now();  // Capture the current time
  }

  setStatus(BT::NodeStatus::RUNNING);
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if(elapsed >= 0.8*timeout_){
    // std::cout<< "Going to timeout soon, " << timeout_ - elapsed  << "secs left" << std::endl;
  }
  // std::cout<< "elapsed: " << elapsed << "retry_count: " << retry_count_ << std::endl;
  // std::cout<< "current_child_idx_: " << current_child_idx_ << "number_of_retries_: " << number_of_retries_ << std::endl;

  while ((current_child_idx_ < children_count && retry_count_ <= number_of_retries_) && (elapsed < timeout_)) {
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
            if (retry_count_ < number_of_retries_ && elapsed < timeout_) {
              // halt first child and tick second child in next iteration
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
              // reset node and return failure when max retries has been exceeded
              halt();
              return BT::NodeStatus::FAILURE;
            }
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
            retry_count_++;
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
  std::cout<< "Returning failure, retry count: " << retry_count_ << "elapsed: " << elapsed << std::endl;
  return BT::NodeStatus::FAILURE;
}

void RecoveryNodeWithTimeout::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
  start_time_ = std::chrono::steady_clock::time_point{};
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNodeWithTimeout>("RecoveryNodeWithTimeout");
}
