#include "wallfollowing.h"

Wallfollowing::Wallfollowing()
{
    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &Wallfollowing::laserScanCallback, this);
    m_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

Point& Wallfollowing::determinePredictedCarPosition(ProcessedTrack& processedTrack)
{
}

Point& Wallfollowing::determineTargetCarPosition(ProcessedTrack& processedTrack)
{
}

void Wallfollowing::followWalls(ProcessedTrack& processedTrack, double delta_time)
{
}

void Wallfollowing::handleLaserPointcloud(std::vector<Point>& pointcloud, double delta_time)
{
}

std::vector<Point>& Wallfollowing::getScanAsCartesian(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
}

void Wallfollowing::publishDriveParameters(double angle, double velocity)
{
    drive_msgs::drive_param message;
    message.velocity = velocity;
    message.angle = angle;
    m_drive_parameters_publisher.publish(message);
}

void Wallfollowing::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    std::vector<Point> pointcloud;
    double scan_time = laserscan->header.stamp.toSec();
    if (std::abs(scan_time - m_last_scan_time) > 0.0001 && scan_time > m_last_scan_time)
    {
        double delta_time = scan_time - m_last_scan_time;
        // getScanAsCartesian();
        // handleLaserPointcloud(scan_message, delta_time);
    }
    m_last_scan_time = scan_time;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wallfollowing5");
    Wallfollowing wallfollowing;
    ros::spin();
    return EXIT_SUCCESS;
}