#ifndef HUSKY_BT__IDLE_MONITOR_HPP_
#define HUSKY_BT__IDLE_MONITOR_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class IdleMonitor : public BT::ConditionNode {
public:
  IdleMonitor(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

#endif
