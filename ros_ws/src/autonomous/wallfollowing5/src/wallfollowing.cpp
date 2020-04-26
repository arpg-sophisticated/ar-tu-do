#include "wallfollowing.h"

Wallfollowing::Wallfollowing()
{
    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &Wallfollowing::laserScanCallback, this);
    m_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

Point Wallfollowing::determinePredictedCarPosition(ProcessedTrack& processedTrack)
{
    double prediction_distance = std::min(0.1 + m_speed_controller.getLastDeterminedSpeed() * 0.35, 2.0);
    return Point{ 0, prediction_distance };
}

Point Wallfollowing::determineTargetCarPosition(ProcessedTrack& processed_track, Point& predicted_position,
                                                Point& car_position)
{
    Point left_point = processed_track.left_circle.getClosestPoint(predicted_position);
    Point right_point = processed_track.right_circle.getClosestPoint(predicted_position);
    Point central_point = Point{ (left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2 };

    double track_width = std::abs(left_point.x - right_point.x);

    Point target_position = central_point;

    Point upper_point;
    if (processed_track.curve_type == CURVE_TYPE_LEFT)
    {
        // target_position = Point{central_point.x + 0.0, central_point.y};
        if (predicted_position.y > processed_track.curve_entry.y)
        {
            upper_point = processed_track.upper_circle.getClosestPoint(predicted_position);
            target_position = Point{ upper_point.x - track_width / 2, (left_point.y + right_point.y) / 2 };
        }
    }
    else if (processed_track.curve_type == CURVE_TYPE_RIGHT)
    {
        // target_position = Point{central_point.x - 0.0, central_point.y}
        if (predicted_position.y > processed_track.curve_entry.y)
        {
            upper_point = processed_track.upper_circle.getClosestPoint(predicted_position);
            target_position = Point{ upper_point.x + track_width / 2, (left_point.y + right_point.y) / 2 };
        }
    }

    std::vector<Point> rviz_closest_points = { left_point, right_point };
    m_rviz_geometry.showLineInRviz(2, rviz_closest_points, ColorRGBA{ 1, 1, 1, 0.3 }, 0.005);

    if (Config::USE_CIRCLE_TANGENTS)
    {
        double safety_distance = 1.5 * car_config::REAR_WHEEL_DISTANCE;
        std::vector<Point> left_tangent_points;
        std::vector<Point> right_tangent_points;
        // left curve
        if (processed_track.left_circle.getCenter().x < 0 && processed_track.right_circle.getCenter().x < 0 &&
            (processed_track.left_circle.getRadius() < 1000 || processed_track.right_circle.getRadius() < 1000))
        {
            Circle safety_circle = Circle(processed_track.left_circle.getCenter(),
                                          processed_track.left_circle.getRadius() + safety_distance);
            left_tangent_points = safety_circle.calcTangents(car_position);
            if (left_tangent_points.size() > 0)
            {
                target_position = left_tangent_points.front();
            }
        }
        // right curve
        if (processed_track.left_circle.getCenter().x > 0 and processed_track.right_circle.getCenter().x > 0 &&
            (processed_track.left_circle.getRadius() < 1000 || processed_track.right_circle.getRadius() < 1000))
        {
            Circle safety_circle = Circle(processed_track.right_circle.getCenter(),
                                          processed_track.right_circle.getRadius() + safety_distance);
            right_tangent_points = safety_circle.calcTangents(car_position);
            if (right_tangent_points.size() > 0)
            {
                target_position = right_tangent_points.front();
            }
        }

        if (left_tangent_points.size() > 0)
        {
            std::vector<Point> rviz_tangent_points = { car_position, target_position };
            m_rviz_geometry.showLineInRviz(20, rviz_tangent_points, ColorRGBA{ 1, 1, 1, 1.0 });
        }
        else
        {
            m_rviz_geometry.deleteMarker(20);
        }
        if (right_tangent_points.size() > 0)
        {
            std::vector<Point> rviz_tangent_points = { car_position, target_position };
            m_rviz_geometry.showLineInRviz(21, rviz_tangent_points, ColorRGBA{ 0.5, 0.5, 0.5, 1.0 });
        }
        else
        {
            m_rviz_geometry.deleteMarker(21);
        }
    }

    return target_position;
}

void Wallfollowing::followWalls(ProcessedTrack& processed_track, double delta_time)
{
    double speed = m_speed_controller.calcSpeed(processed_track);

    Point car_position = { 0, 0 };
    Point predicted_position = determinePredictedCarPosition(processed_track);
    Point target_position = determineTargetCarPosition(processed_track, predicted_position, car_position);
    double angle =
        m_steering_controller.determineSteeringAngle(car_position, predicted_position, target_position, delta_time);

    std::vector<Point> rviz_predicted_car_points = { car_position, predicted_position };
    m_rviz_geometry.showLineInRviz(3, rviz_predicted_car_points, ColorRGBA{ 1, 1, 1, 0.3 }, 0.005);
    std::vector<Point> rviz_target_points = { predicted_position, target_position };
    m_rviz_geometry.showLineInRviz(4, rviz_target_points, ColorRGBA{ 1, 0.4, 0, 1 });

    publishDriveParameters(angle, speed);
}

void Wallfollowing::handleLaserPointcloud(std::vector<Point>& pointcloud, double delta_time)
{
    ProcessedTrack processed_track;
    if (m_process_track.processTrack(&processed_track, pointcloud))
    {
        followWalls(processed_track, delta_time);
    }
    else
    {
        std::cerr << "processed_track not valid" << std::endl;
    }
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
    double t_start = ros::Time::now().toSec();
    if (std::abs(scan_time - m_last_scan_time) > 0.0001 && scan_time > m_last_scan_time)
    {
        double delta_time = scan_time - m_last_scan_time;
        std::vector<Point> pointcloud;
        getScanAsCartesian(&pointcloud, laserscan);
        handleLaserPointcloud(pointcloud, delta_time);
        double t_end = ros::Time::now().toSec();
        // std::cout << "time since last scan: " << delta_time << "s scan execution time: " << t_end - t_start << "s" <<
        // std::endl;
    }
    m_last_scan_time = scan_time;
}

void Wallfollowing::clusterCallback(const sensor_msgs::PointCloud2::ConstPtr& cluster)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wallfollowing");
    Wallfollowing wallfollowing;
    ros::spin();
    return EXIT_SUCCESS;
}