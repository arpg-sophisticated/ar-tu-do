#include "camera_to_pointcloud.h"
#include <ros/ros.h>

#define TOPIC_INPUT_CLOUD "/scan/camera/depthcloud"
#define TOPIC_OUTPUT_CLOUD "/scan/fusion/cartesian"

CameraToPointCloud::CameraToPointCloud()
    : m_private_node_handle("~")
{
    std::string topicInputCloud;
    std::string topicOutputCloud;
    if (!this->m_private_node_handle.getParamCached("topic_input_cloud", topicInputCloud))
        topicInputCloud = TOPIC_INPUT_CLOUD;

    if (!this->m_private_node_handle.getParamCached("topic_output_cloud", topicOutputCloud))
        topicOutputCloud = TOPIC_OUTPUT_CLOUD;

    this->m_input_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicInputCloud, 1, &CameraToPointCloud::input_callback,
                                                          this);
    this->m_voxel_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicOutputCloud, 1, true);
}

void CameraToPointCloud::input_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_to_pointcloud");
    CameraToPointCloud camera_to_pointcloud;
    ros::spin();
    return EXIT_SUCCESS;
}
