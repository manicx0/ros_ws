#include "husky_bt/idle_monitor.hpp"

IdleMonitor::IdleMonitor(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList IdleMonitor::providedPorts() { return {}; }

BT::NodeStatus IdleMonitor::tick() {
  bool has_goal = false;
  if (!config().blackboard->get<bool>("has_goal", has_goal) || !has_goal) {
    config().blackboard->set<bool>("idle", true);
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set<bool>("idle", false);
  return BT::NodeStatus::SUCCESS;
}
