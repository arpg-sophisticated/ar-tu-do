#include "sensor_msgs/PointCloud2.h"
#include <cam_lidar_fusion/cam_lidar_fusion_node.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

CamLidarFusion::CamLidarFusion()
    : m_private_node_handle("~")
{
    std::string topicLidar;
    std::string topicCam;
    std::string topicFused;

    if (!this->m_private_node_handle.getParamCached("topic_input_lidar", topicLidar))
        topicLidar = "/scan/lidar/cartesian";

    if (!this->m_private_node_handle.getParamCached("topic_input_cam", topicCam))
        topicCam = "/scan/cam/cartesian";

    if (!this->m_private_node_handle.getParamCached("topic_output_fused", topicFused))
        topicFused = "/scan/fusion/cartesian";

    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicLidar, 1, &CamLidarFusion::lidar_input_callback, this);
    this->m_cam_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicCam, 1, &CamLidarFusion::cam_input_callback, this);
    this->m_fused_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicFused, 1, true);
};

void CamLidarFusion::cam_input_callback(const sensor_msgs::PointCloud2::ConstPtr& cam)
{
    pcl::fromROSMsg(*cam, this->last_camera_pointcloud);
    // fuse();
};

void CamLidarFusion::lidar_input_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
    pcl::fromROSMsg(*lidar, this->last_lidar_pointcloud);
    fuse();
};

void CamLidarFusion::fuse()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    output_pointcloud->header.frame_id = this->last_lidar_pointcloud.header.frame_id;

    // WARNING
    // incredibly sophisticated fusion algorithm:
    if (this->last_lidar_pointcloud.size() > 0)
    {
        output_pointcloud->header.frame_id = this->last_lidar_pointcloud.header.frame_id;
        *output_pointcloud += this->last_lidar_pointcloud;
    }
    else if (this->last_camera_pointcloud.size() > 0)
    {
        output_pointcloud->header.frame_id = this->last_camera_pointcloud.header.frame_id;
    }

    if (this->last_camera_pointcloud.size() > 0)
    {
        *output_pointcloud += this->last_camera_pointcloud;
    }
    // END WARNING

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*output_pointcloud, output);

    // Publish the data
    this->m_fused_publisher.publish(output);
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_lidar_fusion");
    CamLidarFusion wallSeparation;
    ros::spin();
    return EXIT_SUCCESS;
}
