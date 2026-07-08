#ifndef HUSKY_NAV__PATH_PLANNER_HPP_
#define HUSKY_NAV__PATH_PLANNER_HPP_

#include <vector>
#include "husky_nav/pure_pursuit.hpp"

class SimplePathPlanner {
public:
    static std::vector<Point2D> generateStraightLine(const Pose2D& start, const Point2D& goal, double resolution = 0.1) {
        std::vector<Point2D> path;
        double dist = std::hypot(goal.x - start.x, goal.y - start.y);
        int num_steps = static_cast<int>(dist / resolution);
        
        for (int i = 0; i <= num_steps; ++i) {
            double t = static_cast<double>(i) / num_steps;
            Point2D pt;
            pt.x = start.x + t * (goal.x - start.x);
            pt.y = start.y + t * (goal.y - start.y);
            path.push_back(pt);
        }
        return path;
    }
};

#endif  // HUSKY_NAV__PATH_PLANNER_HPP_
