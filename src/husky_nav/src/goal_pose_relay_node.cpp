#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalPoseRelay : public rclcpp::Node {
public:
  GoalPoseRelay() : Node("goal_pose_relay_node") {
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pub_->publish(*msg);
      });
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_waypoints", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseRelay>());
  rclcpp::shutdown();
  return 0;
}
