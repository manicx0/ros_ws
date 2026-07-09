#include "husky_bt/emergency_stop_condition.hpp"

EmergencyStopCondition::EmergencyStopCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList EmergencyStopCondition::providedPorts() { return {}; }

BT::NodeStatus EmergencyStopCondition::tick() {
  bool emergency_stop = false;
  if (config().blackboard->get<bool>("emergency_stop", emergency_stop) && emergency_stop) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
