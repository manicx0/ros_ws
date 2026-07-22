#ifndef HUSKY_NAV__VFH_PLANNER_HPP_
#define HUSKY_NAV__VFH_PLANNER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "sensor_msgs/msg/laser_scan.hpp"

struct VFHSector {
    double angle_center;
    double min_range;
    bool blocked;
};

struct VFHValley {
    double start_angle;
    double end_angle;
    double center_angle;
    double width;
};

struct VFHOutput {
    double steering_angle;
    double linear_speed;
    bool goal_reached;
    bool path_blocked;
};

class VFHPlanner {
public:
    VFHPlanner() = default;

    std::vector<VFHSector> buildHistogram(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        int num_sectors,
        double obstacle_range,
        double safety_margin)
    {
        std::vector<VFHSector> sectors(num_sectors);
        double sector_width = 2.0 * M_PI / num_sectors;

        for (int i = 0; i < num_sectors; ++i) {
            sectors[i].angle_center = -M_PI + (i + 0.5) * sector_width;
            sectors[i].min_range = scan->range_max;
            sectors[i].blocked = false;
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max) {
                continue;
            }

            double angle = scan->angle_min + i * scan->angle_increment;
            int sector_idx = static_cast<int>((angle + M_PI) / sector_width);
            sector_idx = std::clamp(sector_idx, 0, num_sectors - 1);

            if (range < sectors[sector_idx].min_range) {
                sectors[sector_idx].min_range = range;
            }
        }

        for (auto& sector : sectors) {
            if (sector.min_range < obstacle_range + safety_margin) {
                sector.blocked = true;
            }
        }

        return sectors;
    }

    std::vector<VFHValley> findValleys(
        const std::vector<VFHSector>& sectors,
        double min_gap_width)
    {
        std::vector<VFHValley> valleys;
        int num_sectors = sectors.size();
        double sector_width = 2.0 * M_PI / num_sectors;

        int start_idx = -1;
        for (int i = 0; i < num_sectors; ++i) {
            if (!sectors[i].blocked) {
                if (start_idx == -1) {
                    start_idx = i;
                }
            } else {
                if (start_idx != -1) {
                    int end_idx = i - 1;
                    double width = (end_idx - start_idx + 1) * sector_width;
                    if (width >= min_gap_width) {
                        VFHValley valley;
                        valley.start_angle = sectors[start_idx].angle_center - sector_width / 2.0;
                        valley.end_angle = sectors[end_idx].angle_center + sector_width / 2.0;
                        valley.center_angle = (valley.start_angle + valley.end_angle) / 2.0;
                        valley.width = width;
                        valleys.push_back(valley);
                    }
                    start_idx = -1;
                }
            }
        }

        if (start_idx != -1) {
            int end_idx = num_sectors - 1;
            double width = (end_idx - start_idx + 1) * sector_width;
            if (width >= min_gap_width) {
                VFHValley valley;
                valley.start_angle = sectors[start_idx].angle_center - sector_width / 2.0;
                valley.end_angle = sectors[end_idx].angle_center + sector_width / 2.0;
                valley.center_angle = (valley.start_angle + valley.end_angle) / 2.0;
                valley.width = width;
                valleys.push_back(valley);
            }
        }

        return valleys;
    }

    double selectValley(
        const std::vector<VFHValley>& valleys,
        double goal_bearing)
    {
        if (valleys.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        double best_angle = valleys[0].center_angle;
        double best_cost = std::numeric_limits<double>::max();

        for (const auto& valley : valleys) {
            double angle_diff = std::abs(normalizeAngle(valley.center_angle - goal_bearing));
            double width_penalty = 1.0 / valley.width;
            double cost = angle_diff + 0.3 * width_penalty;

            if (cost < best_cost) {
                best_cost = cost;
                best_angle = valley.center_angle;
            }
        }

        return best_angle;
    }

    double computeSpeed(
        const std::vector<VFHSector>& sectors,
        double steering_angle,
        double max_speed,
        double min_speed)
    {
        double sector_width = 2.0 * M_PI / sectors.size();
        int steering_sector = static_cast<int>((steering_angle + M_PI) / sector_width);
        steering_sector = std::clamp(steering_sector, 0, static_cast<int>(sectors.size()) - 1);

        double min_range = sectors[steering_sector].min_range;
        for (int i = std::max(0, steering_sector - 2); i <= std::min(static_cast<int>(sectors.size()) - 1, steering_sector + 2); ++i) {
            if (sectors[i].min_range < min_range) {
                min_range = sectors[i].min_range;
            }
        }

        if (min_range < 0.5) {
            return 0.0;
        }

        double speed = min_speed + (max_speed - min_speed) * std::min(1.0, (min_range - 0.5) / 2.0);
        return speed;
    }

    VFHOutput plan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        double goal_bearing,
        double goal_distance,
        int num_sectors,
        double obstacle_range,
        double safety_margin,
        double min_gap_width,
        double max_speed,
        double min_speed,
        double goal_proximity)
    {
        VFHOutput output;
        output.goal_reached = false;
        output.path_blocked = false;

        if (goal_distance < goal_proximity) {
            output.goal_reached = true;
            output.steering_angle = 0.0;
            output.linear_speed = 0.0;
            return output;
        }

        auto sectors = buildHistogram(scan, num_sectors, obstacle_range, safety_margin);
        auto valleys = findValleys(sectors, min_gap_width);

        if (valleys.empty()) {
            output.path_blocked = true;
            output.steering_angle = 0.0;
            output.linear_speed = 0.0;
            return output;
        }

        double steering = selectValley(valleys, goal_bearing);
        if (std::isnan(steering)) {
            output.path_blocked = true;
            output.steering_angle = 0.0;
            output.linear_speed = 0.0;
            return output;
        }

        output.steering_angle = steering;
        output.linear_speed = computeSpeed(sectors, steering, max_speed, min_speed);
        return output;
    }

private:
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif  // HUSKY_NAV__VFH_PLANNER_HPP_
