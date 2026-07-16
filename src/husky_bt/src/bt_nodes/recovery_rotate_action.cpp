#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class RecoveryRotate : public BT::StatefulActionNode {
public:
  RecoveryRotate(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
      recovery_pub_ = node_->create_publisher<std_msgs::msg::Bool>("recovery_active", 10);
    }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    start_time_ = node_->now();
    publishRecoveryActive(true);
    publishRotation();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    auto elapsed = (node_->now() - start_time_).seconds();
    if (elapsed >= 3.0) {
      stopRotation();
      publishRecoveryActive(false);
      return BT::NodeStatus::SUCCESS;
    }
    publishRotation();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    stopRotation();
    publishRecoveryActive(false);
  }

private:
  void publishRotation() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.twist.angular.z = 0.4;
    cmd_pub_->publish(cmd);
  }

  void stopRotation() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.twist.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishRecoveryActive(bool active) {
    std_msgs::msg::Bool msg;
    msg.data = active;
    recovery_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_pub_;
  rclcpp::Time start_time_;
};
