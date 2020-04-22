#pragma once

#include "car_config.h"
#include "circle.h"
#include "physical_properties.h"
#include "sensor_msgs/LaserScan.h"
#include "speed_controller.h"
#include <vector>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";
constexpr const char* TOPIC_LASER_SCAN = "/scan";

enum CurveType
{
    CURVE_TYPE_RIGHT; CURVE_TYPE_LEFT;
}

struct ProcessedTrack
{
    std::vector<Point> left_wall;
    std::vector<Point> right_wall;
    std::vector<Point> upper_wall;

    Circle left_circle;
    Circle right_circle;
    Circle upper_circle;

    CurveType curve_type;
    double remaining_distance;
}

class Wallfollowing
{
    private:
    SpeedController m_speed_controller;
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_laserscan_subscriber;

    public:
    Wallfollowing();

    int findLeftRightBorder(std::vector<Point> pointcloud);
    Point determinePredictedCarPosition(ProcessedTrack processedTrack);
    Point determineTargetCarPosition(ProcessedTrack processedTrack);
    void followWalls(ProcessedTrack processedTrack);
    std::vector<Point> pointcloud getScanAsCartesian(const sensor_msgs::LaserScan::ConstPtr& laserscan);
    void handleLaserPointcloud(std::vector<Point> pointcloud);

    void publishDriveParameters(double angle, double velocity);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
};