#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class RecoveryRotate : public BT::SyncActionNode {
public:
  RecoveryRotate(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.twist.angular.z = 0.4;
    cmd_pub_->publish(cmd);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
};
