#include <functional>
#include <cmath>
#include <vector>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "husky_nav/pure_pursuit.hpp"
#include "tf2/utils.hpp"

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    this->declare_parameter<double>("lookahead_dist", 1.0);
    this->declare_parameter<double>("linear_speed", 0.5);
    this->declare_parameter<double>("avoidance_distance", 2.0);
    this->declare_parameter<double>("wall_angular_threshold", 0.5);
    this->declare_parameter<double>("avoidance_speed_factor", 0.5);
    this->declare_parameter<double>("min_gap_width", 0.3);
    this->declare_parameter<double>("rotation_speed", 0.4);
    this->declare_parameter<double>("rotation_tolerance", 0.1);
    this->declare_parameter<double>("rotation_timeout", 10.0);

    this->get_parameter("lookahead_dist", lookahead_dist_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("avoidance_distance", avoidance_distance_);
    this->get_parameter("wall_angular_threshold", wall_angular_threshold_);
    this->get_parameter("avoidance_speed_factor", avoidance_speed_factor_);
    this->get_parameter("min_gap_width", min_gap_width_);
    this->get_parameter("rotation_speed", rotation_speed_);
    this->get_parameter("rotation_tolerance", rotation_tolerance_);
    this->get_parameter("rotation_timeout", rotation_timeout_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "platform/odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "global_path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_2d", 10, std::bind(&PurePursuitNode::scanCallback, this, std::placeholders::_1));

    rotation_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "rotation_goal", 10, std::bind(&PurePursuitNode::rotationCallback, this, std::placeholders::_1));

    recovery_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "recovery_active", 10, std::bind(&PurePursuitNode::recoveryCallback, this, std::placeholders::_1));

    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10, std::bind(&PurePursuitNode::emergencyStopCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&PurePursuitNode::controlLoop, this));
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

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_.clear();
    for (const auto& pose : msg->poses) {
      current_path_.push_back({pose.pose.position.x, pose.pose.position.y});
    }
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
      current_path_.clear();
      recovery_active_ = false;
      rotation_active_ = false;
    }
  }

  enum class ObstacleType { CLEAR, POLE, WALL };

  ObstacleType classifyObstacle(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                 double& obstacle_angle, double& min_distance) {
    if (!scan) return ObstacleType::CLEAR;

    // Check forward cone (-0.5 to +0.5 rad) instead of single ray
    double forward_cone_half = 0.5;  // radians
    int forward_idx = static_cast<int>((0.0 - scan->angle_min) / scan->angle_increment);
    
    if (forward_idx < 0 || forward_idx >= static_cast<int>(scan->ranges.size())) {
      return ObstacleType::CLEAR;
    }

    // Check if any ray in the forward cone detects an obstacle
    bool forward_blocked = false;
    int cone_half_indices = static_cast<int>(forward_cone_half / scan->angle_increment);
    
    for (int i = forward_idx - cone_half_indices; i <= forward_idx + cone_half_indices; ++i) {
      if (i >= 0 && i < static_cast<int>(scan->ranges.size())) {
        if (scan->ranges[i] < avoidance_distance_ && scan->ranges[i] > scan->range_min) {
          forward_blocked = true;
          break;
        }
      }
    }

    if (!forward_blocked) {
      return ObstacleType::CLEAR;
    }

    min_distance = std::numeric_limits<double>::max();
    double closest_angle = 0.0;
    double max_cluster_width = 0.0;

    int cluster_start = -1;
    for (int i = 0; i < static_cast<int>(scan->ranges.size()); ++i) {
      double range = scan->ranges[i];
      bool is_obstacle = (range < avoidance_distance_ && range > scan->range_min);

      if (is_obstacle) {
        if (cluster_start == -1) cluster_start = i;
        if (range < min_distance) {
          min_distance = range;
          closest_angle = scan->angle_min + i * scan->angle_increment;
        }
      } else {
        if (cluster_start >= 0) {
          double cluster_width = (i - cluster_start) * scan->angle_increment;
          if (cluster_width > max_cluster_width) {
            max_cluster_width = cluster_width;
          }
          cluster_start = -1;
        }
      }
    }

    if (cluster_start >= 0) {
      double cluster_width = (static_cast<int>(scan->ranges.size()) - cluster_start) * scan->angle_increment;
      if (cluster_width > max_cluster_width) {
        max_cluster_width = cluster_width;
      }
    }

    if (min_distance >= avoidance_distance_) {
      return ObstacleType::CLEAR;
    }

    obstacle_angle = closest_angle;

    if (max_cluster_width >= wall_angular_threshold_) {
      return ObstacleType::WALL;
    }
    return ObstacleType::POLE;
  }

  double findGapAngle(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!scan) return 0.0;

    std::vector<std::pair<double, double>> gaps;

    int gap_start = -1;
    for (int i = 0; i < static_cast<int>(scan->ranges.size()); ++i) {
      double range = scan->ranges[i];
      bool is_free = (range >= avoidance_distance_ || range <= scan->range_min);

      if (is_free) {
        if (gap_start == -1) gap_start = i;
      } else {
        if (gap_start >= 0) {
          double gap_width = (i - gap_start) * scan->angle_increment;
          if (gap_width >= min_gap_width_) {
            double gap_center_angle = scan->angle_min + (gap_start + i) / 2.0 * scan->angle_increment;
            gaps.push_back({gap_center_angle, gap_width});
          }
          gap_start = -1;
        }
      }
    }

    if (gap_start >= 0) {
      int i = static_cast<int>(scan->ranges.size());
      double gap_width = (i - gap_start) * scan->angle_increment;
      if (gap_width >= min_gap_width_) {
        double gap_center_angle = scan->angle_min + (gap_start + i) / 2.0 * scan->angle_increment;
        gaps.push_back({gap_center_angle, gap_width});
      }
    }

    if (gaps.empty()) return 0.0;

    double best_gap_angle = gaps[0].first;
    double best_score = std::numeric_limits<double>::max();

    for (const auto& gap : gaps) {
      double angle = gap.first;
      double width = gap.second;
      double score = std::abs(angle) / width;
      if (score < best_score) {
        best_score = score;
        best_gap_angle = angle;
      }
    }

    return best_gap_angle;
  }

  void controlLoop() {
    if (emergency_stop_) {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = this->now();
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }
    if (recovery_active_) return;
    if (current_path_.empty() && !rotation_active_) return;

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
    } else {
      auto lookahead = findLookaheadPoint(robot_pose_, current_path_, lookahead_dist_);

      double dx = lookahead.x - robot_pose_.x;
      double dy = lookahead.y - robot_pose_.y;
      double local_y = -dx * sin(robot_pose_.theta) + dy * cos(robot_pose_.theta);
      double curvature = 2.0 * local_y / (lookahead_dist_ * lookahead_dist_);

      cmd.twist.linear.x  = linear_speed_;
      cmd.twist.angular.z = linear_speed_ * curvature;

      if (latest_scan_) {
        double obstacle_angle = 0.0;
        double min_distance = 0.0;
        ObstacleType obstacle_type = classifyObstacle(latest_scan_, obstacle_angle, min_distance);

        if (obstacle_type == ObstacleType::WALL) {
          cmd.twist.linear.x = 0.0;
          cmd.twist.angular.z = 0.0;
        } else if (obstacle_type == ObstacleType::POLE) {
          double gap_angle = findGapAngle(latest_scan_);
          double avoidance_curvature = 2.0 * std::sin(gap_angle) / (lookahead_dist_ * lookahead_dist_);
          cmd.twist.angular.z = avoidance_curvature * linear_speed_;
          cmd.twist.linear.x = linear_speed_ * avoidance_speed_factor_;
        }
      }

      if (!current_path_.empty()) {
        auto& last = current_path_.back();
        double dx = last.x - robot_pose_.x;
        double dy = last.y - robot_pose_.y;
        double dist_to_end = std::sqrt(dx * dx + dy * dy);
        if (dist_to_end < 0.3) {
          cmd.twist.linear.x = 0.0;
          cmd.twist.angular.z = 0.0;
          current_path_.clear();
        }
      }
    }

    cmd_pub_->publish(cmd);
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  double lookahead_dist_;
  double linear_speed_;
  double avoidance_distance_;
  double wall_angular_threshold_;
  double avoidance_speed_factor_;
  double min_gap_width_;
  double rotation_speed_;
  double rotation_tolerance_;
  double rotation_timeout_;
  Pose2D robot_pose_{0.0, 0.0, 0.0};
  std::vector<Point2D> current_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  bool rotation_active_ = false;
  double rotation_goal_ = 0.0;
  rclcpp::Time rotation_start_time_;
  double rotation_start_yaw_ = 0.0;
  bool recovery_active_ = false;
  bool emergency_stop_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rotation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}