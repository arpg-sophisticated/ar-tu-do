#pragma once

#include "dbscan.h"
#include <dynamic_reconfigure/server.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <voxel_classifier/voxel_classifierConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

constexpr const char* TOPIC_VOXEL_ = "/scan/voxels";
constexpr const char* TOPIC_CLUSTER_ = "/scan/cluster";

class VoxelClassifier
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;
    ros::Subscriber m_voxel_subscriber;
    ros::Publisher m_marker_publisher;
    ros::Publisher m_cluster_publisher;
    std::string m_frame;

    dynamic_reconfigure::Server<voxel_classifier::voxel_classifierConfig> m_dyn_cfg_server;
    double m_epsilon;
    int m_minimum_points;
    double m_color_weight;

    public:
    VoxelClassifier();
    void voxel_callback(pcl::PointCloud<pcl::PointXYZRGBL> lidar);
};
