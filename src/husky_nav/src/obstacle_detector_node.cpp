#include <functional>
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

    // 4. Check for obstacles
    if (!cloud->empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Obstacle points detected: %zu", cloud->size());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
