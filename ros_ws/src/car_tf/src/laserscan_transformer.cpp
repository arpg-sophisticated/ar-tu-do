#include "laserscan_transformer.h"
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

LaserscanTransformer::LaserscanTransformer()
    : m_private_node_handle("~")
{

    std::string topicLaserScan;
    std::string topicOutput;

    if (!this->m_private_node_handle.getParamCached("topic_input", topicLaserScan))
        topicLaserScan = TOPIC_LASER_SCAN;

    if (!this->m_private_node_handle.getParamCached("topic_output", topicOutput))
        topicOutput = TOPIC_LASER_SCAN_POINTCLOUD;

    if (!this->m_private_node_handle.getParamCached("base_link", m_base_link))
        m_base_link = MODEL_BASE_LINK;

    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(topicLaserScan, 100, &LaserscanTransformer::scanCallback, this);
    m_pointcloud_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicOutput, 100, false);
}

void LaserscanTransformer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    if (!m_listener.waitForTransform(laserscan->header.frame_id, m_base_link,
                                     laserscan->header.stamp +
                                         ros::Duration().fromSec(laserscan->ranges.size() * laserscan->time_increment),
                                     ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud2 pointcloud;
    m_projector.transformLaserScanToPointCloud(m_base_link, *laserscan, pointcloud, m_listener);
    m_pointcloud_publisher.publish(pointcloud);
}
