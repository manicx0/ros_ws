#ifndef HUSKY_NAV__OBSTACLE_DETECTOR_HPP_
#define HUSKY_NAV__OBSTACLE_DETECTOR_HPP_

#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

// Lightweight structural matching for your obstacle output
struct Obstacle {
    geometry_msgs::msg::Point centroid;
    geometry_msgs::msg::Vector3 bounding_box;
};

#endif  // HUSKY_NAV__OBSTACLE_DETECTOR_HPP_
