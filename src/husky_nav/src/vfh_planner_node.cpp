#include <functional>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "husky_nav/vfh_planner.hpp"
#include "tf2/utils.hpp"

class VFHPlannerNode : public rclcpp::Node {
public:
  VFHPlannerNode() : Node("vfh_planner_node") {
    this->declare_parameter<double>("max_speed", 0.45);
    this->declare_parameter<double>("min_speed", 0.15);
    this->declare_parameter<double>("max_angular_speed", 0.8);
    this->declare_parameter<int>("num_sectors", 72);
    this->declare_parameter<double>("min_gap_width", 0.5);
    this->declare_parameter<double>("obstacle_range", 3.0);
    this->declare_parameter<double>("goal_proximity", 0.5);
    this->declare_parameter<double>("safety_margin", 0.3);
    this->declare_parameter<double>("rotation_speed", 0.4);
    this->declare_parameter<double>("rotation_tolerance", 0.1);
    this->declare_parameter<double>("rotation_timeout", 10.0);

    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("min_speed", min_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("num_sectors", num_sectors_);
    this->get_parameter("min_gap_width", min_gap_width_);
    this->get_parameter("obstacle_range", obstacle_range_);
    this->get_parameter("goal_proximity", goal_proximity_);
    this->get_parameter("safety_margin", safety_margin_);
    this->get_parameter("rotation_speed", rotation_speed_);
    this->get_parameter("rotation_tolerance", rotation_tolerance_);
    this->get_parameter("rotation_timeout", rotation_timeout_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "platform/odom", 10, std::bind(&VFHPlannerNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_waypoints", 10, std::bind(&VFHPlannerNode::goalCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_2d", 10, std::bind(&VFHPlannerNode::scanCallback, this, std::placeholders::_1));

    rotation_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "rotation_goal", 10, std::bind(&VFHPlannerNode::rotationCallback, this, std::placeholders::_1));

    recovery_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "recovery_active", 10, std::bind(&VFHPlannerNode::recoveryCallback, this, std::placeholders::_1));

    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10, std::bind(&VFHPlannerNode::emergencyStopCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&VFHPlannerNode::controlLoop, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_.x = msg->pose.pose.position.x;
    robot_pose_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    robot_pose_.theta = tf2::impl::getYaw(q);
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    has_goal_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  void rotationCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    rotation_goal_ = msg->data;
    rotation_active_ = true;
    rotation_start_time_ = this->now();
    rotation_start_yaw_ = robot_pose_.theta;
  }

  void recoveryCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    recovery_active_ = msg->data;
  }

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_stop_ = msg->data;
    if (emergency_stop_) {
      has_goal_ = false;
      recovery_active_ = false;
      rotation_active_ = false;
    }
  }

  void controlLoop() {
    if (emergency_stop_) {
      publishZeroVelocity();
      return;
    }
    if (recovery_active_) return;
    if (!has_goal_ && !rotation_active_) return;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();

    if (rotation_active_) {
      double elapsed = (this->now() - rotation_start_time_).seconds();
      double yaw_delta = normalizeAngle(robot_pose_.theta - rotation_start_yaw_);
      double remaining = rotation_goal_ - yaw_delta;

      if (std::abs(remaining) < rotation_tolerance_ || elapsed > rotation_timeout_) {
        rotation_active_ = false;
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
      } else {
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = (remaining > 0 ? 1.0 : -1.0) * rotation_speed_;
      }
    } else if (has_goal_) {
      double dx = goal_.x - robot_pose_.x;
      double dy = goal_.y - robot_pose_.y;
      double goal_distance = std::hypot(dx, dy);

      if (goal_distance < goal_proximity_) {
        has_goal_ = false;
        publishZeroVelocity();
        publishEmptyPath();
        return;
      }

      double goal_bearing_world = std::atan2(dy, dx);
      double goal_bearing_robot = normalizeAngle(goal_bearing_world - robot_pose_.theta);

      VFHOutput output = vfh_planner_.plan(
        latest_scan_,
        goal_bearing_robot,
        goal_distance,
        num_sectors_,
        obstacle_range_,
        safety_margin_,
        min_gap_width_,
        max_speed_,
        min_speed_,
        goal_proximity_);

      if (output.goal_reached) {
        has_goal_ = false;
        publishZeroVelocity();
        publishEmptyPath();
        return;
      }

      if (output.path_blocked) {
        publishZeroVelocity();
        return;
      }

      cmd.twist.linear.x = output.linear_speed;
      cmd.twist.angular.z = std::clamp(
        output.linear_speed * std::sin(output.steering_angle) / 1.0,
        -max_angular_speed_, max_angular_speed_);

      publishVisualizationPath(output.steering_angle);
    }

    cmd_pub_->publish(cmd);
  }

  void publishZeroVelocity() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishEmptyPath() {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "odom";
    path_pub_->publish(path);
  }

  void publishVisualizationPath(double steering_angle) {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "odom";

    double x = robot_pose_.x;
    double y = robot_pose_.y;
    double theta = robot_pose_.theta + steering_angle;

    for (int i = 0; i < 10; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "odom";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      path.poses.push_back(pose);

      x += 0.2 * std::cos(theta);
      y += 0.2 * std::sin(theta);
    }

    path_pub_->publish(path);
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  double max_speed_;
  double min_speed_;
  double max_angular_speed_;
  int num_sectors_;
  double min_gap_width_;
  double obstacle_range_;
  double goal_proximity_;
  double safety_margin_;
  double rotation_speed_;
  double rotation_tolerance_;
  double rotation_timeout_;

  VFHPlanner vfh_planner_;

  struct { double x = 0.0; double y = 0.0; double theta = 0.0; } robot_pose_;
  struct { double x = 0.0; double y = 0.0; } goal_;

  bool has_goal_ = false;
  bool rotation_active_ = false;
  bool recovery_active_ = false;
  bool emergency_stop_ = false;

  double rotation_goal_ = 0.0;
  rclcpp::Time rotation_start_time_;
  double rotation_start_yaw_ = 0.0;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rotation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFHPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
