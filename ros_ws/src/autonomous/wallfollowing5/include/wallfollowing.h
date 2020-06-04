#pragma once

#include "car_config.h"
#include "circle.h"
#include "config.h"
#include "geometric_math.h"
#include "physical_properties.h"
#include "process_track.h"
#include "rviz_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "speed_controller.h"
#include "steering_controller.h"
#include <cmath>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";
constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_VOXEL = "/scan/voxels";
constexpr const char* TOPIC_CLUSTER = "/scan/clusters";

class Wallfollowing
{
    private:
    SpeedController m_speed_controller;
    SteeringController m_steering_controller;
    ProcessTrack m_process_track;
    RvizGeometry m_rviz_geometry;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_laserscan_subscriber;
    ros::Subscriber m_voxel_subscriber;
    ros::Subscriber m_cluster_subscriber;
    ros::Publisher m_drive_parameters_publisher;

    double m_last_scan_time;

    public:
    Wallfollowing();

    Point determinePredictedCarPosition(ProcessedTrack& processedTrack);
    Point determineTargetCarPositionCircleTangents(ProcessedTrack& processedTrack, Point& predicted_position,
                                                   Point& car_position);
    void followWalls(ProcessedTrack& processedTrack, double delta_time);
    void getScanAsCartesian(std::vector<Point>* storage, const sensor_msgs::LaserScan::ConstPtr& laserscan);
    void handleLaserPointcloud(std::vector<Point>& pointcloud, double delta_time);

    void publishDriveParameters(double angle, double velocity);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
    void clusterCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cluster);
    void voxelCallback(const sensor_msgs::PointCloud2::ConstPtr& voxel_msg);
};