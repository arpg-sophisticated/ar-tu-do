#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class CamLidarFusion
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;

    ros::Subscriber m_cam_subscriber;
    ros::Subscriber m_lidar_subscriber;

    ros::Publisher m_fused_publisher;

    pcl::PointCloud<pcl::PointXYZ> last_camera_pointcloud;
    pcl::PointCloud<pcl::PointXYZ> last_lidar_pointcloud;

    void fuse();

    public:
    CamLidarFusion();
    void cam_input_callback(const sensor_msgs::PointCloud2::ConstPtr& cam);
    void lidar_input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);
};
