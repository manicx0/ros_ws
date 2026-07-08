#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class NavigateToGoal : public BT::StatefulActionNode {
public:
  NavigateToGoal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
      goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_waypoints", 10);
    }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
             BT::OutputPort<bool>("goal_reached") };
  }

  BT::NodeStatus onStart() override {
    getInput("goal", goal_);
    goal_pub_->publish(goal_);
    obstacle_detected_ = false;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (distanceToGoal() < 0.5) {
      setOutput("goal_reached", true);
      return BT::NodeStatus::SUCCESS;
    }
    if (obstacle_detected_) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { stopRobot(); }

private:
  double distanceToGoal() { return 0.0; } // Simulated loop tracking distance
  void stopRobot() {}
  
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  bool obstacle_detected_ = false;
};
