#include "husky_bt/stop_and_wait.hpp"

StopAndWait::StopAndWait(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node) {
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
}

BT::PortsList StopAndWait::providedPorts() { return {}; }

BT::NodeStatus StopAndWait::tick() {
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::SUCCESS;
}
