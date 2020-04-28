#pragma once

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

class CameraToPointCloud
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;
    ros::Subscriber m_input_subscriber;
    ros::Publisher m_voxel_publisher;

    public:
    void input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar);

    CameraToPointCloud();
};
