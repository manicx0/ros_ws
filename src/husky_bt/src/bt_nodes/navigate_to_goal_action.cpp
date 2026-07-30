#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "husky_msgs/msg/goal_event.hpp"
#include "husky_msgs/msg/goal_reached.hpp"

class NavigateToGoal : public BT::StatefulActionNode {
public:
  NavigateToGoal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
      goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_waypoints", 10);
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
      clear_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("global_path", 10);
      event_pub_ = node_->create_publisher<husky_msgs::msg::GoalEvent>("/fleet/goal_events", 10);
      vfh_reached_sub_ = node_->create_subscription<husky_msgs::msg::GoalReached>(
        "vfh_goal_reached", 10,
        std::bind(&NavigateToGoal::vfhReachedCallback, this, std::placeholders::_1));
    }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
             BT::OutputPort<bool>("goal_reached") };
  }

  BT::NodeStatus onStart() override {
    getInput("goal", goal_);
    goal_pub_->publish(goal_);
    vfh_goal_reached_ = false;
    start_time_ = node_->now();
    config().blackboard->set<bool>("goal_reached", false);
    config().blackboard->set<bool>("mission_active", true);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (vfh_goal_reached_ && msg_time_ > start_time_) {
      setOutput("goal_reached", true);
      onSuccess();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { stopRobot(); }

private:
  void vfhReachedCallback(const husky_msgs::msg::GoalReached::SharedPtr msg) {
    msg_time_ = msg->stamp;
    vfh_goal_reached_ = msg->reached;
  }

  void stopRobot() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void onSuccess() {
    stopRobot();

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = node_->now();
    empty_path.header.frame_id = "odom";
    clear_path_pub_->publish(empty_path);

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
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr clear_path_pub_;
  rclcpp::Publisher<husky_msgs::msg::GoalEvent>::SharedPtr event_pub_;
  rclcpp::Subscription<husky_msgs::msg::GoalReached>::SharedPtr vfh_reached_sub_;
  bool vfh_goal_reached_ = false;
  rclcpp::Time start_time_;
  rclcpp::Time msg_time_;
};
