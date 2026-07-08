#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "husky_nav/pure_pursuit.hpp"
#include "tf2/utils.hpp"

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    // Declare parameters
    this->declare_parameter<double>("lookahead_dist", 1.0);
    this->declare_parameter<double>("linear_speed", 0.5);

    this->get_parameter("lookahead_dist", lookahead_dist_);
    this->get_parameter("linear_speed", linear_speed_);

    // Subscriptions & Publisher
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
      
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "global_path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Timer loop for tracking execution
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

  void controlLoop() {
    if (current_path_.empty()) return;

    auto lookahead = findLookaheadPoint(robot_pose_, current_path_, lookahead_dist_);
    
    double dx = lookahead.x - robot_pose_.x;
    double dy = lookahead.y - robot_pose_.y;
    double local_y = -dx * sin(robot_pose_.theta) + dy * cos(robot_pose_.theta);
    double curvature = 2.0 * local_y / (lookahead_dist_ * lookahead_dist_);
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_speed_;
    cmd.angular.z = linear_speed_ * curvature; 
    cmd_pub_->publish(cmd);
  }

  double lookahead_dist_;
  double linear_speed_;
  Pose2D robot_pose_{0.0, 0.0, 0.0};
  std::vector<Point2D> current_path_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
