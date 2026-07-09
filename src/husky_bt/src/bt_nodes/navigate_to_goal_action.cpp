#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "husky_msgs/msg/goal_event.hpp"

class NavigateToGoal : public BT::StatefulActionNode {
public:
  NavigateToGoal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
      goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_waypoints", 10);
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      event_pub_ = node_->create_publisher<husky_msgs::msg::GoalEvent>("/fleet/goal_events", 10);
      odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&NavigateToGoal::odomCallback, this, std::placeholders::_1));
    }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
             BT::OutputPort<bool>("goal_reached") };
  }

  BT::NodeStatus onStart() override {
    getInput("goal", goal_);
    goal_pub_->publish(goal_);
    obstacle_detected_ = false;
    config().blackboard->set<bool>("mission_active", true);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    bool obstacle = false;
    if (config().blackboard->get<bool>("obstacle_detected", obstacle) && obstacle) {
      stopRobot();
      return BT::NodeStatus::RUNNING;
    }

    if (distanceToGoal() < 0.5) {
      setOutput("goal_reached", true);
      onSuccess();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { stopRobot(); }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  double distanceToGoal() {
    double dx = goal_.pose.position.x - current_x_;
    double dy = goal_.pose.position.y - current_y_;
    return std::sqrt(dx * dx + dy * dy);
  }

  void stopRobot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void onSuccess() {
    husky_msgs::msg::GoalEvent event;
    event.type = husky_msgs::msg::GoalEvent::GOAL_REACHED;
    event.robot_id = node_->get_namespace();
    event.goal = goal_;
    event_pub_->publish(event);

    config().blackboard->set<bool>("has_goal", false);
    config().blackboard->set<bool>("idle", true);
    config().blackboard->set<bool>("mission_active", false);
  }

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<husky_msgs::msg::GoalEvent>::SharedPtr event_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  bool obstacle_detected_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
};
