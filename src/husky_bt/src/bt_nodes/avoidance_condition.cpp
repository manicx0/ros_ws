#include "husky_bt/avoidance_condition.hpp"

AvoidanceEnabledCondition::AvoidanceEnabledCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList AvoidanceEnabledCondition::providedPorts() { return {}; }

BT::NodeStatus AvoidanceEnabledCondition::tick() {
  bool avoidance_enabled = false;
  if (!config().blackboard->get<bool>("avoidance_enabled", avoidance_enabled) || !avoidance_enabled) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
