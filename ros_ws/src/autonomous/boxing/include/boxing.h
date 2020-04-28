#pragma once

#include "rviz_geometry_publisher.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <unordered_map>

#include <dynamic_reconfigure/server.h>
#include <boxing/boxingConfig.h>

constexpr const char* TOPIC_INPUT_POINTCLOUD = "/scan/lidar/cartesian";
constexpr const char* TOPIC_VISUALIZATION = "/wall_separation_visualization";
constexpr const char* TOPIC_VOXEL_ = "/scan/voxels";

constexpr const char* LIDAR_FRAME = "laser";

class Boxing
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;
    ros::Subscriber m_input_subscriber;
    ros::Publisher m_voxel_publisher;
    sensor_msgs::PointCloud2 voxelsCloud;

    RvizGeometryPublisher m_debug_geometry;
    dynamic_reconfigure::Server<boxing::boxingConfig> m_dyn_cfg_server;

    float m_voxel_size;

    public:
    Boxing();
    void input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
};
