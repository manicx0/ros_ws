#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "husky_nav/path_planner.hpp"

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode() : Node("path_planner_node") {
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_waypoints", 10, std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));
      
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10, std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Point2D goal{msg->pose.position.x, msg->pose.position.y};
    auto points = SimplePathPlanner::generateStraightLine(current_pose_, goal);

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "odom";

    for (const auto& pt : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  Pose2D current_pose_{0.0, 0.0, 0.0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
