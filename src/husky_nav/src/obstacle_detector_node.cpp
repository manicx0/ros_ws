#include <functional>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class ObstacleDetectorNode : public rclcpp::Node {
public:
  ObstacleDetectorNode() : Node("obstacle_detector_node") {
    this->declare_parameter("scan_angle_min", -M_PI);
    this->declare_parameter("scan_angle_max", M_PI);
    this->declare_parameter("scan_angle_increment", 0.01);
    this->declare_parameter("scan_range_min", 0.1);
    this->declare_parameter("scan_range_max", 10.0);
    this->declare_parameter("scan_num_readings", 628);

    scan_angle_min_ = this->get_parameter("scan_angle_min").as_double();
    scan_angle_max_ = this->get_parameter("scan_angle_max").as_double();
    scan_angle_increment_ = this->get_parameter("scan_angle_increment").as_double();
    scan_range_min_ = this->get_parameter("scan_range_min").as_double();
    scan_range_max_ = this->get_parameter("scan_range_max").as_double();
    scan_num_readings_ = this->get_parameter("scan_num_readings").as_int();

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, std::bind(&ObstacleDetectorNode::pointcloudCallback, this, std::placeholders::_1));

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_2d", 10);
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 1. PassThrough: Filter ground and roof height limits
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 1.5);
    pass.filter(*cloud);

    // 2. VoxelGrid Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud);

    // 3. Publish filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud, filtered_msg);
    filtered_msg.header = msg->header;
    filtered_cloud_pub_->publish(filtered_msg);

    // 4. Convert to LaserScan and publish
    publishScan(*cloud, msg->header);

    // 5. Debug log
    if (!cloud->empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Obstacle points detected: %zu", cloud->size());
    }
  }

  void publishScan(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::LaserScan scan;
    scan.header = header;
    scan.angle_min = scan_angle_min_;
    scan.angle_max = scan_angle_max_;
    scan.angle_increment = scan_angle_increment_;
    scan.range_min = scan_range_min_;
    scan.range_max = scan_range_max_;
    scan.ranges.resize(scan_num_readings_, scan_range_max_);

    for (const auto& point : cloud.points) {
      double range = std::sqrt(point.x * point.x + point.y * point.y);
      if (range < scan_range_min_ || range > scan_range_max_) continue;

      double angle = std::atan2(point.y, point.x);
      if (angle < scan_angle_min_ || angle > scan_angle_max_) continue;

      int index = static_cast<int>((angle - scan_angle_min_) / scan_angle_increment_);
      if (index >= 0 && index < scan_num_readings_) {
        if (range < scan.ranges[index]) {
          scan.ranges[index] = range;
        }
      }
    }

    scan_pub_->publish(scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  double scan_angle_min_;
  double scan_angle_max_;
  double scan_angle_increment_;
  double scan_range_min_;
  double scan_range_max_;
  int scan_num_readings_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
