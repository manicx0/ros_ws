#include <functional>
#include <behaviortree_cpp/condition_node.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class StuckCheck : public BT::SimpleConditionNode {
public:
  StuckCheck(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SimpleConditionNode(name, std::bind(&StuckCheck::checkStuck, this), config), node_(node) {
    stuck_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "stuck", 10,
      std::bind(&StuckCheck::stuckCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts() { return {}; }

private:
  void stuckCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_stuck_ = msg->data;
  }

  BT::NodeStatus checkStuck() {
    config().blackboard->set<bool>("stuck", is_stuck_);
    return is_stuck_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stuck_sub_;
  bool is_stuck_ = false;
};
