#pragma once
#include <../../wall_separation/include/voxel.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

constexpr const char *TOPIC_VOXEL_ = "/scan/voxels";

constexpr const char *LIDAR_FRAME = "laser";

class VoxelClassifier {
private:
  ros::NodeHandle m_node_handle;
  ros::Subscriber m_voxel_subscriber;

public:
  VoxelClassifier();
  void voxel_callback(const sensor_msgs::PointCloud2::ConstPtr &lidar);
};
