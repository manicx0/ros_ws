#include <behaviortree_cpp/action_node.h>
#include "rclcpp/rclcpp.hpp"

class RecoveryFailed : public BT::SyncActionNode {
public:
  RecoveryFailed(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {
      node_->declare_parameter<int>("max_recovery_attempts", 3);
      max_attempts_ = node_->get_parameter("max_recovery_attempts").as_int();
    }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    int attempts = 0;
    config().blackboard->get<int>("recovery_attempts", attempts);
    attempts++;
    config().blackboard->set<int>("recovery_attempts", attempts);
    
    if (attempts >= max_attempts_) {
      config().blackboard->set<bool>("has_goal", false);
      config().blackboard->set<bool>("recovery_failed", true);
      RCLCPP_ERROR(node_->get_logger(), "Recovery failed after %d attempts - aborting mission", attempts);
      return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_WARN(node_->get_logger(), "Recovery attempt %d/%d failed - retrying", attempts, max_attempts_);
    config().blackboard->set<bool>("stuck", false);
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  int max_attempts_;
};
