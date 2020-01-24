#pragma once
#include "rviz_geometry_publisher.h"
#include "voxel.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <unordered_map>

constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_VISUALIZATION = "/wallfollowing_visualization";

constexpr const char* LIDAR_FRAME = "laser";

class WallSeparation
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_lidar_subscriber;
    RvizGeometryPublisher m_debug_geometry;
    std::unordered_map<std::string, Voxel> m_voxels; // TODO: Replace string representation with something proper
    public:
    WallSeparation();
    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar);
};
