#ifndef HUSKY_NAV__PURE_PURSUIT_HPP_
#define HUSKY_NAV__PURE_PURSUIT_HPP_

#include <vector>
#include <cmath>

struct Point2D {
    double x;
    double y;
};

struct Pose2D {
    double x;
    double y;
    double theta;
};

inline double distance(const Pose2D& p1, const Point2D& p2) {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

inline Point2D findLookaheadPoint(const Pose2D& robot, const std::vector<Point2D>& path, double lookahead_dist) {
    if (path.empty()) return {robot.x, robot.y};
    
    // Default to the last point if no point is far enough
    Point2D lookahead = path.back();
    for (const auto& pt : path) {
        if (distance(robot, pt) >= lookahead_dist) {
            lookahead = pt;
            break;
        }
    }
    return lookahead;
}

#endif  // HUSKY_NAV__PURE_PURSUIT_HPP_
