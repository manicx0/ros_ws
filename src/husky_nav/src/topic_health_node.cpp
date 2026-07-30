#include <functional>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TopicHealthNode : public rclcpp::Node {
public:
  TopicHealthNode() : Node("topic_health_node") {
    this->declare_parameter<double>("timeout_seconds", 2.0);
    this->get_parameter("timeout_seconds", timeout_seconds_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "platform/odom", 10,
      std::bind(&TopicHealthNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_2d", 10,
      std::bind(&TopicHealthNode::scanCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      std::bind(&TopicHealthNode::cmdVelCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_waypoints", 10,
      std::bind(&TopicHealthNode::goalCallback, this, std::placeholders::_1));

    health_pub_ = this->create_publisher<std_msgs::msg::String>("topic_health", 10);

    timer_ = this->create_wall_timer(
      1s, std::bind(&TopicHealthNode::publishHealth, this));

    RCLCPP_INFO(this->get_logger(), "Topic health monitor started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
    last_odom_ = this->now();
    odom_count_++;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/) {
    last_scan_ = this->now();
    scan_count_++;
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr /*msg*/) {
    last_cmd_vel_ = this->now();
    cmd_vel_count_++;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/) {
    last_goal_ = this->now();
    goal_count_++;
  }

  void publishHealth() {
    auto now = this->now();
    std_msgs::msg::String health_msg;

    bool odom_ok = (now - last_odom_).seconds() < timeout_seconds_;
    bool scan_ok = (now - last_scan_).seconds() < timeout_seconds_;
    bool cmd_vel_ok = (now - last_cmd_vel_).seconds() < timeout_seconds_;
    bool goal_ok = (now - last_goal_).seconds() < timeout_seconds_;

    std::string status = "Topic Health Status:\n";
    status += "  platform/odom: " + std::string(odom_ok ? "OK" : "STALE") +
              " (" + std::to_string(odom_count_) + " msgs, last " +
              std::to_string((now - last_odom_).seconds()) + "s ago)\n";
    status += "  scan_2d: " + std::string(scan_ok ? "OK" : "STALE") +
              " (" + std::to_string(scan_count_) + " msgs, last " +
              std::to_string((now - last_scan_).seconds()) + "s ago)\n";
    status += "  cmd_vel: " + std::string(cmd_vel_ok ? "OK" : "STALE") +
              " (" + std::to_string(cmd_vel_count_) + " msgs, last " +
              std::to_string((now - last_cmd_vel_).seconds()) + "s ago)\n";
    status += "  goal_waypoints: " + std::string(goal_ok ? "OK" : "STALE") +
              " (" + std::to_string(goal_count_) + " msgs, last " +
              std::to_string((now - last_goal_).seconds()) + "s ago)\n";

    health_msg.data = status;
    health_pub_->publish(health_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "odom:%s scan:%s cmd_vel:%s goal:%s",
      odom_ok ? "OK" : "STALE",
      scan_ok ? "OK" : "STALE",
      cmd_vel_ok ? "OK" : "STALE",
      goal_ok ? "OK" : "STALE");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_odom_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_scan_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_cmd_vel_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_goal_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  int odom_count_ = 0;
  int scan_count_ = 0;
  int cmd_vel_count_ = 0;
  int goal_count_ = 0;

  double timeout_seconds_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicHealthNode>());
  rclcpp::shutdown();
  return 0;
}
