#pragma once

#include "rviz_geometry_publisher.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>

#include <boxing/boxingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/transforms.h>

constexpr const char* TOPIC_INPUT_POINTCLOUD = "/scan/lidar/cartesian";
constexpr const char* TOPIC_INPUT_COLORED_CLOUD = "/racer/camera1/depth/points";
constexpr const char* TOPIC_VISUALIZATION = "/wall_separation_visualization";
constexpr const char* TOPIC_VOXEL_ = "/scan/voxels";

constexpr const char* LIDAR_FRAME = "laser";

class Boxing
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;
    tf::TransformListener transform_listener;
    ros::Subscriber m_input_subscriber;
    ros::Subscriber m_colored_point_cloud_subscriber;
    ros::Publisher m_voxel_publisher;

    std::string reference_frame;

    RvizGeometryPublisher m_debug_geometry;
    dynamic_reconfigure::Server<boxing::boxingConfig> m_dyn_cfg_server;

    float m_voxel_size;

    bool m_filter_by_min_score_enabled;
    float m_filter_by_min_score;

    bool m_sor_enabled;
    float m_sor_mean_k;
    float m_sor_stddev_mul_thresh;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_colored_cloud;

    public:
    Boxing();
    void input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
    void colored_input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
};
