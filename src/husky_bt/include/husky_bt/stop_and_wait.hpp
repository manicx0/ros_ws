#ifndef HUSKY_BT__STOP_AND_WAIT_HPP_
#define HUSKY_BT__STOP_AND_WAIT_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class StopAndWait : public BT::SyncActionNode {
public:
  StopAndWait(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

#endif
