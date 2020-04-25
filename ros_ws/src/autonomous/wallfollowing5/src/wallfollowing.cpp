#include "wallfollowing.h"

Wallfollowing::Wallfollowing()
{
    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &Wallfollowing::laserScanCallback, this);
    m_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

Point Wallfollowing::determinePredictedCarPosition(ProcessedTrack& processedTrack)
{
}

Point Wallfollowing::determineTargetCarPosition(ProcessedTrack& processedTrack)
{
}

void Wallfollowing::followWalls(ProcessedTrack& processedTrack, double delta_time)
{
}

void Wallfollowing::handleLaserPointcloud(std::vector<Point>& pointcloud, double delta_time)
{
    ProcessedTrack processed_track;
    process_track.processTrack(&processed_track, pointcloud);
}

void Wallfollowing::getScanAsCartesian(std::vector<Point>* storage, const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    int n = laserscan->ranges.size();
    double skip_angle_range =
        ((laserscan->angle_max - laserscan->angle_min) - GeometricFunctions::toRadians(Config::USABLE_LASER_RANGE)) / 2;
    double angle_start = laserscan->angle_min + skip_angle_range;
    int index_start = skip_angle_range / laserscan->angle_increment;
    int index_end = n - index_start;
    double angle = angle_start;
    for (int i = index_start; i < index_end; i++)
    {
        if (!std::isnan(laserscan->ranges[i]) && !std::isinf(laserscan->ranges[i]))
        {
            Point p;
            p.x = -std::sin(angle) * laserscan->ranges[i];
            p.y = std::cos(angle) * laserscan->ranges[i];
            storage->push_back(p);
        }
        angle += laserscan->angle_increment;
    }
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
    double scan_time = laserscan->header.stamp.toSec();
    if (std::abs(scan_time - m_last_scan_time) > 0.0001 && scan_time > m_last_scan_time)
    {
        double delta_time = scan_time - m_last_scan_time;
        std::vector<Point> pointcloud;
        getScanAsCartesian(&pointcloud, laserscan);
        handleLaserPointcloud(pointcloud, delta_time);
    }
    m_last_scan_time = scan_time;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wallfollowing");
    Wallfollowing wallfollowing;
    ros::spin();
    return EXIT_SUCCESS;
}