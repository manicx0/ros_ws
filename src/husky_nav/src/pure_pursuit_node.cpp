// src/pure_pursuit_node.cpp — key logic
class PurePursuitController : public rclcpp::Node {
  // Subscribes: /odom, /goal_waypoints
  // Publishes:  /cmd_vel

  geometry_msgs::msg::Twist computeVelocity(
      const Pose2D& robot, const std::vector<Point2D>& path) {
    
    // 1. Find lookahead point on path
    auto lookahead = findLookaheadPoint(robot, path, lookahead_dist_);
    
    // 2. Compute curvature κ = 2y / L²
    double dx = lookahead.x - robot.x;
    double dy = lookahead.y - robot.y;
    double local_y = -dx*sin(robot.theta) + dy*cos(robot.theta);
    double curvature = 2.0 * local_y / (lookahead_dist_ * lookahead_dist_);
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_speed_;
    cmd.angular.z = linear_speed_ * curvature;  // ω = v·κ
    return cmd;
  }
};
