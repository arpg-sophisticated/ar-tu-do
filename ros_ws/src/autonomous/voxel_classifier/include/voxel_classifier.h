#pragma once
#include "dbscan.h"
#include "rviz_geometry_publisher.h"
#include "voxel.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

constexpr const char* TOPIC_VOXEL_ = "/scan/voxels";
constexpr const char* TOPIC_VISUALIZATION_Cluster = "/wall_separation_visualization_cluster";
constexpr const char* LIDAR_FRAME = "laser";

class VoxelClassifier
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_voxel_subscriber;
    std::vector<Voxel> voxels;
    RvizGeometryPublisher m_debug_geometry;
    ros::Publisher m_marker_publisher;
    const std::string m_frame;

    public:
    VoxelClassifier();
    void voxel_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
    // std::vector<Point> transformPoints();
    void printResults(vector<Voxel>& points, int num_points);
    void clearRects();
};
