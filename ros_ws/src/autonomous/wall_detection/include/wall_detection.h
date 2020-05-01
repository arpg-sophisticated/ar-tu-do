#pragma once

#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <unordered_map>

constexpr const char *TOPIC_VOXEL_ = "/scan/cluster";
constexpr const char *TOPIC_WALLS_ = "/obstacles/walls";
constexpr const char *TOPIC_OBSTACLES_ = "/obstacles/obstacles";

constexpr const char *LIDAR_FRAME = "laser";
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class WallDetection {
private:
  ros::NodeHandle m_node_handle;
  ros::Subscriber m_voxel_subscriber;
  ros::Publisher m_wall_publisher;
  ros::Publisher m_obstacles_publisher;
  const std::string m_frame;
  std::vector<PointCloud> listOfClusters;
  std::pair<int, int>
  voxelMaximaIDs(std::unordered_map<int, std::vector<pcl::PointXYZI> *>);
  void publishWall(std::vector<pcl::PointXYZI> *wallLeft,
                   std::vector<pcl::PointXYZI> *wallRight);
  void publishObstacles(
      std::unordered_map<int, std::vector<pcl::PointXYZI> *> mapClusters,
      std::pair<int, int> wallIDs);
  std::string frameID;

public:
  WallDetection();
  void wallDetection_callback(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &inputVoxels);
  PointCloud pointCloudOfID(const PointCloud::ConstPtr &inputVoxels, int ID);
};
