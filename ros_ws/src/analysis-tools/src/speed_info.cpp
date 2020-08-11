#include "speed_info.h"

using namespace std;

SpeedInfo::SpeedInfo()
{
    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &SpeedInfo::laserScanCallback, this);
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &SpeedInfo::controlledDriveParametersCallback, this);
    m_gazebo_state_telemetry_subscriber =
        m_node_handle.subscribe<drive_msgs::gazebo_state_telemetry>(TOPIC_GAZEBO_STATE_TELEMETRY, 1,
                                                                    &SpeedInfo::gazeboStateTelemetryCallback, this);
    m_max_speed_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_MAX_SPEED, 1);
}

void SpeedInfo::publishMaxSpeed(std::vector<Point>& pointcloud)
{
    ProcessedTrack processed_track;
    if (m_process_track.processTrack(&processed_track, pointcloud))
    {
        double acceleration = PhysicalProperties::getAcceleration();
        double dynamic_friction = PhysicalProperties::getDynamicFriction();
        double speed = calcSpeed(processed_track, acceleration, dynamic_friction);
        std_msgs::Float64 publish_speed;
        publish_speed.data = speed;
        m_max_speed_publisher.publish(publish_speed);
    }
    else
    {
        std::cerr << "speed info: processed_track not valid" << std::endl;
    }
}

double SpeedInfo::calcMaxCurveSpeed(double radius, double dynamic_friction)
{
    return sqrt(dynamic_friction * 9.81 * radius);
}

double SpeedInfo::calcMaxSpeed(double distance, double target_speed, double acceleration)
{
    return sqrt((2 * distance * acceleration * acceleration + m_current_speed * m_current_speed * acceleration +
                 target_speed * target_speed * acceleration) /
                (acceleration + acceleration));
}

double SpeedInfo::calcBrakingDistance(double distance, double target_speed, double acceleration)
{
    return (2 * distance * acceleration + m_current_speed * m_current_speed - target_speed * target_speed) /
        (2 * acceleration + 2 * acceleration);
}

double SpeedInfo::calcSpeed(ProcessedTrack& processed_track, double acceleration, double dynamic_friction)
{
    double remaining_distance = processed_track.curve_entry.y;
    double radius = min(processed_track.left_circle.getRadius(), processed_track.right_circle.getRadius());
    double speed = calcMaxCurveSpeed(radius, dynamic_friction);
    if (processed_track.curve_type != CURVE_TYPE_STRAIGHT)
    {
        double safety_margin = 0.25;
        if (remaining_distance < 5)
        {
            safety_margin = 0.05 * remaining_distance;
        }
        double target_speed = calcMaxCurveSpeed(processed_track.upper_circle.getRadius(), dynamic_friction);
        double braking_distance = calcBrakingDistance(remaining_distance, target_speed, acceleration) + safety_margin;
        if (remaining_distance > braking_distance)
        {
            speed = min(calcMaxSpeed(remaining_distance, target_speed, acceleration), speed);
        }
        else
        {
            speed = target_speed;
        }
    }
    m_last_determined_speed = speed;
    return speed;
}

void SpeedInfo::getScanAsCartesian(std::vector<Point>* storage, const sensor_msgs::LaserScan::ConstPtr& laserscan)
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

void SpeedInfo::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    std::vector<Point> pointcloud;
    getScanAsCartesian(&pointcloud, laserscan);
    publishMaxSpeed(pointcloud);
}

void SpeedInfo::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    if (!m_is_simulated)
    {
        m_current_speed = parameters->velocity;
    }
}

void SpeedInfo::gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr& parameters)
{
    m_current_speed = parameters->car_speed;
    m_is_simulated = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_info");
    SpeedInfo speed_controller;
    ros::spin();
    return EXIT_SUCCESS;
}