#ifndef HUSKY_BT__EMERGENCY_STOP_CONDITION_HPP_
#define HUSKY_BT__EMERGENCY_STOP_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class EmergencyStopCondition : public BT::ConditionNode {
public:
  EmergencyStopCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

#endif
