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
#include "husky_msgs/msg/goal_reached.hpp"
#include "husky_nav/vfh_planner.hpp"
#include "tf2/utils.hpp"

class VFHPlannerNode : public rclcpp::Node {
public:
  VFHPlannerNode() : Node("vfh_planner_node"), start_time_(this->now()) {
    this->declare_parameter<double>("max_speed", 0.45);
    this->declare_parameter<double>("min_speed", 0.15);
    this->declare_parameter<double>("max_angular_speed", 0.8);
    this->declare_parameter<double>("heading_gain", 2.0);
    this->declare_parameter<int>("num_sectors", 72);
    this->declare_parameter<double>("min_gap_width", 0.5);
    this->declare_parameter<double>("obstacle_range", 3.0);
    this->declare_parameter<double>("goal_proximity", 0.5);
    this->declare_parameter<double>("safety_margin", 0.3);
    this->declare_parameter<double>("rotation_speed", 0.4);
    this->declare_parameter<double>("rotation_tolerance", 0.1);
    this->declare_parameter<double>("rotation_timeout", 10.0);
    this->declare_parameter<double>("scan_timeout", 3.0);
    this->declare_parameter<double>("facing_goal_threshold", 0.3);

    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("min_speed", min_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("heading_gain", heading_gain_);
    this->get_parameter("num_sectors", num_sectors_);
    this->get_parameter("min_gap_width", min_gap_width_);
    this->get_parameter("obstacle_range", obstacle_range_);
    this->get_parameter("goal_proximity", goal_proximity_);
    this->get_parameter("safety_margin", safety_margin_);
    this->get_parameter("rotation_speed", rotation_speed_);
    this->get_parameter("rotation_tolerance", rotation_tolerance_);
    this->get_parameter("rotation_timeout", rotation_timeout_);
    this->get_parameter("scan_timeout", scan_timeout_);
    this->get_parameter("facing_goal_threshold", facing_goal_threshold_);

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
    goal_reached_pub_ = this->create_publisher<husky_msgs::msg::GoalReached>("vfh_goal_reached", 10);

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
    facing_goal_ = true;
    publishGoalReached(false);
    RCLCPP_INFO(get_logger(), "GOAL received: (%.2f, %.2f)", goal_.x, goal_.y);
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
      RCLCPP_WARN(get_logger(), "EXIT: emergency_stop active");
      return;
    }
    if (recovery_active_) {
      RCLCPP_WARN(get_logger(), "EXIT: recovery_active, yielding");
      return;
    }
    if (!has_goal_ && !rotation_active_) {
      RCLCPP_DEBUG(get_logger(), "EXIT: no goal and no rotation active");
      return;
    }

    // Check if we have scan data
    bool has_scan = (latest_scan_ != nullptr);
    bool scan_timeout_exceeded = (this->now() - start_time_).seconds() > scan_timeout_;

    // If no scan data but timeout exceeded, drive blind with reduced speed
    if (!has_scan && scan_timeout_exceeded) {
      if (has_goal_) {
        double dx = goal_.x - robot_pose_.x;
        double dy = goal_.y - robot_pose_.y;
        double goal_distance = std::hypot(dx, dy);

        if (goal_distance < goal_proximity_) {
          has_goal_ = false;
          publishGoalReached(true);
          publishZeroVelocity();
          publishEmptyPath();
          return;
        }

        double goal_bearing_world = std::atan2(dy, dx);
        double goal_bearing_robot = normalizeAngle(goal_bearing_world - robot_pose_.theta);

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();

        if (facing_goal_) {
          if (std::abs(goal_bearing_robot) > facing_goal_threshold_) {
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = std::clamp(
              heading_gain_ * goal_bearing_robot,
              -max_angular_speed_, max_angular_speed_);
            RCLCPP_INFO(get_logger(),
              "FACING (blind): bearing=%.2f rad  cmd_angular=%.2f",
              goal_bearing_robot, cmd.twist.angular.z);
            cmd_pub_->publish(cmd);
            publishVisualizationPath(goal_bearing_robot);
            return;
          }
          facing_goal_ = false;
          RCLCPP_INFO(get_logger(), "FACING: aligned at bearing=%.2f rad", goal_bearing_robot);
        }

        cmd.twist.linear.x = min_speed_;
        cmd.twist.angular.z = std::clamp(
          heading_gain_ * goal_bearing_robot,
          -max_angular_speed_, max_angular_speed_);
        RCLCPP_WARN(get_logger(),
          "BLIND_DRIVE: bearing_robot=%.2f rad  dist=%.2f  cmd=(%.2f, %.2f)",
          goal_bearing_robot, goal_distance, cmd.twist.linear.x, cmd.twist.angular.z);
        cmd_pub_->publish(cmd);

        publishVisualizationPath(goal_bearing_robot);
      }
      return;
    }

    // If no scan data and timeout not exceeded, wait
    if (!has_scan) {
      publishZeroVelocity();
      return;
    }

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
        publishGoalReached(true);
        publishZeroVelocity();
        publishEmptyPath();
        return;
      }

      double goal_bearing_world = std::atan2(dy, dx);
      double goal_bearing_robot = normalizeAngle(goal_bearing_world - robot_pose_.theta);
      RCLCPP_INFO(get_logger(),
        "POSE: (%.2f, %.2f, %.2f)  GOAL: (%.2f, %.2f)  DIST: %.2f  BW: %.2f  BR: %.2f",
        robot_pose_.x, robot_pose_.y, robot_pose_.theta,
        goal_.x, goal_.y, goal_distance,
        goal_bearing_world, goal_bearing_robot);

      if (facing_goal_) {
        if (std::abs(goal_bearing_robot) > facing_goal_threshold_) {
          cmd.twist.linear.x = 0.0;
          cmd.twist.angular.z = std::clamp(
            heading_gain_ * goal_bearing_robot,
            -max_angular_speed_, max_angular_speed_);
          RCLCPP_INFO(get_logger(),
            "FACING: bearing=%.2f rad  cmd_angular=%.2f",
            goal_bearing_robot, cmd.twist.angular.z);
          cmd_pub_->publish(cmd);
          publishVisualizationPath(goal_bearing_robot);
          return;
        }
        facing_goal_ = false;
        RCLCPP_INFO(get_logger(), "FACING: aligned at bearing=%.2f rad", goal_bearing_robot);
      }

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
        publishGoalReached(true);
        publishZeroVelocity();
        publishEmptyPath();
        return;
      }

      if (output.path_blocked) {
        RCLCPP_WARN(get_logger(), "VFH: path_blocked — stopping");
        publishZeroVelocity();
        return;
      }

      cmd.twist.linear.x = output.linear_speed;
      cmd.twist.angular.z = std::clamp(
        heading_gain_ * output.steering_angle,
        -max_angular_speed_, max_angular_speed_);
      RCLCPP_INFO(get_logger(),
        "VFH_OUT: steer=%.2f rad  speed=%.2f  goal_reached=%d  path_blocked=%d",
        output.steering_angle, output.linear_speed,
        output.goal_reached, output.path_blocked);
      RCLCPP_INFO(get_logger(),
        "CMD: (%.2f, %.2f)", cmd.twist.linear.x, cmd.twist.angular.z);

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
    RCLCPP_DEBUG(get_logger(), "CMD: zero velocity (stopped)");
  }

  void publishGoalReached(bool reached) {
    husky_msgs::msg::GoalReached msg;
    msg.reached = reached;
    msg.stamp = this->now();
    goal_reached_pub_->publish(msg);
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
  double heading_gain_;
  int num_sectors_;
  double min_gap_width_;
  double obstacle_range_;
  double goal_proximity_;
  double safety_margin_;
  double rotation_speed_;
  double rotation_tolerance_;
  double rotation_timeout_;
  double scan_timeout_;
  double facing_goal_threshold_;

  rclcpp::Time start_time_;
  VFHPlanner vfh_planner_;

  struct { double x = 0.0; double y = 0.0; double theta = 0.0; } robot_pose_;
  struct { double x = 0.0; double y = 0.0; } goal_;

  bool has_goal_ = false;
  bool facing_goal_ = false;
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
  rclcpp::Publisher<husky_msgs::msg::GoalReached>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFHPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
