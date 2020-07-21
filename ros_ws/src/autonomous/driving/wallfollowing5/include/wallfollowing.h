#pragma once

// clang-format off
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
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
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <dynamic_reconfigure/server.h>
#include <wallfollowing5/wallfollowing5Config.h>
#include <vector>
#include <cmath>
// clang-format on

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";
constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_VOXEL = "/scan/voxels";
constexpr const char* TOPIC_WALLS = "/obstacles/walls";

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
    ros::Subscriber m_walls_subscriber;
    ros::Subscriber m_lidar_cartesian_subscriber;
    ros::Publisher m_drive_parameters_publisher;

    double m_last_scan_time;

    double m_view_dist_average = 0;

    double m_laser_delta_time = 0;

    std::vector<Point> m_laser_pointcloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_lidar_pointcloud;

    dynamic_reconfigure::Server<wallfollowing5::wallfollowing5Config> m_dyn_cfg_server;
    Config::WallfollowingParams wallfollowing_params;
    Config::SteeringParams steering_params;
    Config::PIDParams pid_params;

    public:
    Wallfollowing();

    Point determineTrackCenter(ProcessedTrack& processed_track, Point& predicted_position);
    bool lineTooCloseToPointcloud(ProcessedTrack& processed_track, Line& line, std::vector<Point>& pointcloud);
    std::pair<Point, Point> determineTargetPathPoint(ProcessedTrack& processed_track, double min_distance,
                                                     double max_distance, double epsilon);

    Point determinePredictedCarPosition(ProcessedTrack& processedTrack);
    Point determineTargetCarPosition(ProcessedTrack& processedTrack, Point& predicted_position);
    void followWalls(ProcessedTrack& processedTrack, double delta_time);
    void getScanAsCartesian(std::vector<Point>* storage, const sensor_msgs::LaserScan::ConstPtr& laserscan);
    void handleLaserPointcloud(std::vector<Point>& pointcloud, double delta_time);
    void handleWallsPointcloud(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wall_pointcloud, double delta_time);

    void publishDriveParameters(double angle, double velocity);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
    void wallsCallback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& walls);
    void voxelCallback(const sensor_msgs::PointCloud2::ConstPtr& voxel_msg);
    void lidarCartesianCallback(const sensor_msgs::PointCloud2::ConstPtr& pointCloud);
};