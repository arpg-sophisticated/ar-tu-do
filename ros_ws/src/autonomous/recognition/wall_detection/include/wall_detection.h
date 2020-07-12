#pragma once

#include "circle.h"
#include <boost/foreach.hpp>
#include <dynamic_reconfigure/server.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <unordered_map>
#include <wall_detection/wall_detectionConfig.h>

constexpr const char* TOPIC_VOXEL_ = "/scan/cluster";
constexpr const char* TOPIC_WALLS_ = "/obstacles/walls";
constexpr const char* TOPIC_OBSTACLES_ = "/obstacles/obstacles";

typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud;

class WallDetection
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;

    ros::Subscriber m_voxel_subscriber;
    ros::Publisher m_wall_publisher;
    ros::Publisher m_obstacles_publisher;
    std::string m_frame;

    dynamic_reconfigure::Server<wall_detection::wall_detectionConfig> m_dyn_cfg_server;
    float m_wall_radius;
    double m_distance_threshold = 0.4;
    uint32_t m_score_threshold = 3;
    double m_minimum_confidence = 0.3;
    bool m_use_prediction_for_walls = true;
    bool m_use_prediction_for_obstacles = true;

    std::pair<int64_t, int64_t> determineWallIDs(std::unordered_map<uint32_t, std::vector<pcl::PointXYZRGBL>*>,
                                                 float radius);
    int64_t findLargestCluster(std::unordered_map<uint32_t, std::vector<pcl::PointXYZRGBL>*> clusters,
                               uint32_t ignoreID);
    void publishWall(std::vector<pcl::PointXYZRGBL>* wallLeft, std::vector<pcl::PointXYZRGBL>* wallRight);
    void publishObstacles(std::unordered_map<uint32_t, std::vector<pcl::PointXYZRGBL>*> mapClusters,
                          std::vector<uint32_t> wallIDs);
    std::pair<std::vector<uint32_t>, std::vector<uint32_t>> addClustersOnRegression(
        std::unordered_map<uint32_t, std::vector<pcl::PointXYZRGBL>*> mapClusters, std::vector<uint32_t> inputIgnoreIDs,
        std::vector<pcl::PointXYZRGBL>* leftWall, std::vector<pcl::PointXYZRGBL>* rightWall);
    Circle fitWall(std::vector<pcl::PointXYZRGBL>* wall);

    public:
    WallDetection();
    void wallDetection_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& inputVoxels);
};
