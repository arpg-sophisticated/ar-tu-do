#pragma once

#include "dbscan.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

constexpr const char* TOPIC_VOXEL_ = "/scan/voxels";
constexpr const char* TOPIC_CLUSTER_ = "/scan/cluster";
constexpr const char* TOPIC_VISUALIZATION_Cluster = "/wall_separation_visualization_cluster";
constexpr const char* LIDAR_FRAME = "laser";

class VoxelClassifier
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_voxel_subscriber;
    ros::Publisher m_marker_publisher;
    ros::Publisher m_cluster_publisher;
    std::string m_frame;

    public:
    VoxelClassifier();
    void voxel_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
    void cluster_publish(std::vector<Point_>* clusters);
};
