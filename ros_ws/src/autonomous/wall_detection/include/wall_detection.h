#pragma once

#include "rviz_geometry_publisher.h"
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

constexpr const char *TOPIC_VOXEL_ = "/scan/cluster";
constexpr const char *TOPIC_VISUALIZATION_Cluster =
    "/wall_separation_visualization_cluster";
constexpr const char *TOPIC_VISUALIZATION_REGION =
    "/wall_separation_visualization_region";
constexpr const char *LIDAR_FRAME = "laser";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class WallDetection {
private:
  ros::NodeHandle m_node_handle;
  ros::Subscriber m_voxel_subscriber;
  RvizGeometryPublisher m_debug_geometry;
  const std::string m_frame;
  std::vector<PointCloud> listOfClusters;

public:
  WallDetection();
  void wallDetection_callback(const PointCloud::ConstPtr &inputVoxels);
  PointCloud pointCloudOfID(const PointCloud::ConstPtr &inputVoxels, int ID);
};
