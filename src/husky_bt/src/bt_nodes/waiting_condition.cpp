#include "husky_bt/waiting_condition.hpp"

WaitingCondition::WaitingCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList WaitingCondition::providedPorts() { return {}; }

BT::NodeStatus WaitingCondition::tick() {
  bool waiting = false;
  if (config().blackboard->get<bool>("waiting", waiting) && waiting) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
