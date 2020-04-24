#pragma once

#include "car_config.h"
#include "circle.h"
#include "physical_properties.h"
#include "process_track.h"
#include "sensor_msgs/LaserScan.h"
#include "speed_controller.h"
#include <cmath>
#include <vector>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";
constexpr const char* TOPIC_LASER_SCAN = "/scan";

class Wallfollowing
{
    private:
    SpeedController m_speed_controller;
    ProcessTrack process_track;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_laserscan_subscriber;
    ros::Publisher m_drive_parameters_publisher;

    double m_last_scan_time;

    public:
    Wallfollowing();

    Point& determinePredictedCarPosition(ProcessedTrack& processedTrack);
    Point& determineTargetCarPosition(ProcessedTrack& processedTrack);
    void followWalls(ProcessedTrack& processedTrack, double delta_time);
    std::vector<Point>& getScanAsCartesian(const sensor_msgs::LaserScan::ConstPtr& laserscan);
    void handleLaserPointcloud(std::vector<Point>& pointcloud, double delta_time);

    void publishDriveParameters(double angle, double velocity);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
};