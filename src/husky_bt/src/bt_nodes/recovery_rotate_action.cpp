#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class RecoveryRotate : public BT::SyncActionNode {
public:
  RecoveryRotate(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.4; // Controlled rotation escape clearing frame
    cmd_pub_->publish(cmd);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
