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
#include <messages_sanity_check.h>
#include <pcl_ros/transforms.h>

constexpr const char* TOPIC_INPUT_POINTCLOUD = "/scan/lidar/cartesian";
constexpr const char* TOPIC_INPUT_COLORED_CLOUD = "/racer/camera1/depth/points";
constexpr const char* TOPIC_VISUALIZATION = "/wall_separation_visualization";
constexpr const char* TOPIC_VOXEL_ = "/scan/voxels";

constexpr const char* LIDAR_FRAME = "laser";

typedef unsigned __int128 uint128_t;

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

    MessageContinuityCheck message_continuity_check;

    float m_voxel_size;
    float m_lidar_percentage;

    bool m_filter_by_min_score_enabled;
    float m_filter_by_min_score;

    bool m_sor_enabled;
    float m_sor_mean_k;
    float m_sor_stddev_mul_thresh;

    bool m_colors_enabled;
    uint32_t m_color_levels;
    bool m_colored_cloud_dirty;
    bool m_adjacent_voxels;
    int m_color_samples;

    float m_maximum_x = 0;
    float m_minimum_x = 0;
    float m_maximum_y = 0;
    float m_minimum_y = 0;
    float m_maximum_z = 0;
    float m_minimum_z = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_colored_cloud;

    void preprocess_colored_cloud();

    public:
    Boxing();
    void input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
    void colored_input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
    static uint64_t get_voxel_id(float x, float y, float z)
    {
        uint64_t voxel_id = 0;
        union {
            float floaty;
            uint32_t inty;
        } float_uint;

        float_uint.floaty = x;
        voxel_id |= static_cast<uint128_t>(float_uint.inty) << 32;
        float_uint.floaty = y;
        voxel_id |= static_cast<uint128_t>(float_uint.inty) << 0;
        // float_uint.floaty = z;
        // voxel_id |= float_uint.inty;
        return voxel_id;
    }
};
