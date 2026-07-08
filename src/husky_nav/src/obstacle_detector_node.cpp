// src/obstacle_detector_node.cpp
// Input:  /velodyne_points (sensor_msgs/PointCloud2)
// Output: /obstacles (custom ObstacleArray msg)
//         /scan_2d   (sensor_msgs/LaserScan, optional)

void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // 1. PassThrough: keep only ground+obstacle height band (0.1m to 1.5m)
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.1, 1.5);
  pass.filter(*cloud);

  // 2. VoxelGrid downsample
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  vg.filter(*cloud);

  // 3. RadiusOutlierRemoval for noise
  // 4. EuclideanClusterExtraction → individual obstacle blobs
  // 5. Publish centroid + bounding box per cluster
}
