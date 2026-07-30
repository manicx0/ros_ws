#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "husky_msgs/msg/robot_state.hpp"

using namespace std::chrono_literals;

class StuckDetectorNode : public rclcpp::Node {
public:
  StuckDetectorNode() : Node("stuck_detector_node") {
    declare_parameter("speed_threshold", 0.1);
    declare_parameter("stuck_threshold", 0.05);
    declare_parameter("grace_period", 2.0);
    declare_parameter("stuck_timeout", 8.0);
    declare_parameter("zero_velocity_stuck_timeout", 10.0);

    speed_threshold_ = get_parameter("speed_threshold").as_double();
    stuck_threshold_ = get_parameter("stuck_threshold").as_double();
    grace_period_ = get_parameter("grace_period").as_double();
    stuck_timeout_ = get_parameter("stuck_timeout").as_double();
    zero_velocity_stuck_timeout_ = get_parameter("zero_velocity_stuck_timeout").as_double();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      std::bind(&StuckDetectorNode::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "platform/odom", 10,
      std::bind(&StuckDetectorNode::odomCallback, this, std::placeholders::_1));

    robot_state_sub_ = create_subscription<husky_msgs::msg::RobotState>(
      "robot_state", 10,
      std::bind(&StuckDetectorNode::robotStateCallback, this, std::placeholders::_1));

    stuck_pub_ = create_publisher<std_msgs::msg::Bool>("stuck", 10);

    timer_ = create_wall_timer(100ms, std::bind(&StuckDetectorNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Stuck detector initialized: speed_threshold=%.2f, stuck_threshold=%.2f, grace_period=%.1f, stuck_timeout=%.1f, zero_velocity_timeout=%.1f",
                speed_threshold_, stuck_threshold_, grace_period_, stuck_timeout_, zero_velocity_stuck_timeout_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    double speed = std::hypot(msg->twist.linear.x, msg->twist.linear.y);
    if (speed > speed_threshold_) {
      if (!commanding_motion_) {
        commanding_motion_ = true;
        command_start_time_ = now();
      }
    } else {
      commanding_motion_ = false;
      stuck_duration_ = 0.0;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    actual_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  }

  void robotStateCallback(const husky_msgs::msg::RobotState::SharedPtr msg) {
    mission_active_ = msg->mission_active;
    if (!mission_active_) {
      zero_velocity_duration_ = 0.0;
    }
  }

  void timerCallback() {
    if (!commanding_motion_) {
      if (is_stuck_) {
        is_stuck_ = false;
        publishStuck(false);
      }
      stuck_duration_ = 0.0;
      
      if (mission_active_ && actual_speed_ < stuck_threshold_) {
        zero_velocity_duration_ += 0.1;
        if (zero_velocity_duration_ >= zero_velocity_stuck_timeout_ && !is_stuck_) {
          is_stuck_ = true;
          publishStuck(true);
          RCLCPP_WARN(get_logger(), "Robot is stuck (zero velocity)! Mission active but actual speed %.3f < %.2f for %.1f seconds",
                      actual_speed_, stuck_threshold_, zero_velocity_duration_);
        }
      } else {
        zero_velocity_duration_ = 0.0;
      }
      return;
    }

    auto elapsed_since_command = (now() - command_start_time_).seconds();
    if (elapsed_since_command < grace_period_) {
      return;
    }

    if (actual_speed_ < stuck_threshold_) {
      stuck_duration_ += 0.1;
      if (stuck_duration_ >= stuck_timeout_ && !is_stuck_) {
        is_stuck_ = true;
        publishStuck(true);
        RCLCPP_WARN(get_logger(), "Robot is stuck! Commanded motion but actual speed %.3f < %.2f for %.1f seconds",
                    actual_speed_, stuck_threshold_, stuck_duration_);
      }
    } else {
      stuck_duration_ = 0.0;
      zero_velocity_duration_ = 0.0;
      if (is_stuck_) {
        is_stuck_ = false;
        publishStuck(false);
        RCLCPP_INFO(get_logger(), "Robot is no longer stuck");
      }
    }
  }

  void publishStuck(bool stuck) {
    auto msg = std_msgs::msg::Bool();
    msg.data = stuck;
    stuck_pub_->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<husky_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double speed_threshold_;
  double stuck_threshold_;
  double grace_period_;
  double stuck_timeout_;
  double zero_velocity_stuck_timeout_;

  bool commanding_motion_ = false;
  bool is_stuck_ = false;
  bool mission_active_ = false;
  double actual_speed_ = 0.0;
  double stuck_duration_ = 0.0;
  double zero_velocity_duration_ = 0.0;
  rclcpp::Time command_start_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StuckDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
