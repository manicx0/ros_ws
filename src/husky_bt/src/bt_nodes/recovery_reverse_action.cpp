#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

class RecoveryReverse : public BT::StatefulActionNode {
public:
  RecoveryReverse(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
      recovery_pub_ = node_->create_publisher<std_msgs::msg::Bool>("recovery_active", 10);
      odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "platform/odom", 10,
        std::bind(&RecoveryReverse::odomCallback, this, std::placeholders::_1));
      scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_2d", 10,
        std::bind(&RecoveryReverse::scanCallback, this, std::placeholders::_1));
      
      node_->declare_parameter<double>("reverse_distance", 1.0);
      node_->declare_parameter<double>("reverse_speed", 0.2);
      node_->declare_parameter<double>("reverse_timeout", 5.0);
      node_->declare_parameter<double>("rear_obstacle_distance", 0.5);
      node_->declare_parameter<double>("rear_cone_half_angle", 0.7854);
      
      node_->get_parameter("reverse_distance", reverse_distance_);
      node_->get_parameter("reverse_speed", reverse_speed_);
      node_->get_parameter("reverse_timeout", reverse_timeout_);
      node_->get_parameter("rear_obstacle_distance", rear_obstacle_distance_);
      node_->get_parameter("rear_cone_half_angle", rear_cone_half_angle_);
    }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    start_time_ = node_->now();
    start_x_ = current_x_;
    start_y_ = current_y_;
    publishRecoveryActive(true);
    publishReverse();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    auto elapsed = (node_->now() - start_time_).seconds();
    double distance_reversed = std::hypot(current_x_ - start_x_, current_y_ - start_y_);
    
    if (elapsed > reverse_timeout_) {
      stopReverse();
      publishRecoveryActive(false);
      if (distance_reversed < reverse_distance_ * 0.5) {
        RCLCPP_WARN(node_->get_logger(), "RecoveryReverse failed - only moved %.2fm of %.2fm target",
                    distance_reversed, reverse_distance_);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
    
    if (distance_reversed >= reverse_distance_) {
      stopReverse();
      publishRecoveryActive(false);
      return BT::NodeStatus::SUCCESS;
    }
    
    if (checkRearObstacle()) {
      stopReverse();
      publishRecoveryActive(false);
      RCLCPP_WARN(node_->get_logger(), "RecoveryReverse aborted - obstacle detected behind robot");
      return BT::NodeStatus::FAILURE;
    }
    
    publishReverse();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    stopReverse();
    publishRecoveryActive(false);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  bool checkRearObstacle() {
    if (!latest_scan_) return false;
    
    double rear_angle = M_PI;
    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
      double angle_diff = std::abs(angle - rear_angle);
      if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
      
      if (angle_diff <= rear_cone_half_angle_) {
        double range = latest_scan_->ranges[i];
        if (range > latest_scan_->range_min && range < rear_obstacle_distance_) {
          return true;
        }
      }
    }
    return false;
  }

  void publishReverse() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.twist.linear.x = -reverse_speed_;
    cmd_pub_->publish(cmd);
  }

  void stopReverse() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.twist.linear.x = 0.0;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Time start_time_;
  
  double reverse_distance_;
  double reverse_speed_;
  double reverse_timeout_;
  double rear_obstacle_distance_;
  double rear_cone_half_angle_;
  
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};
