#include "wallfollowing.h"

Wallfollowing::Wallfollowing()
{
    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &Wallfollowing::laserScanCallback, this);
    m_voxel_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(TOPIC_VOXEL, 1, &Wallfollowing::voxelCallback, this);
    m_walls_subscriber =
        m_node_handle.subscribe<pcl::PointCloud<pcl::PointXYZRGBL>>(TOPIC_WALLS, 1, &Wallfollowing::wallsCallback,
                                                                    this);

    m_lidar_cartesian_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>("/scan/lidar/cartesian", 1,
                                                          &Wallfollowing::lidarCartesianCallback, this);

    m_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);

    m_dyn_cfg_server.setCallback([&](wallfollowing5::wallfollowing5Config& cfg, uint32_t) {
        wallfollowing_params.usable_laser_range = cfg.usable_laser_range;
        wallfollowing_params.target_method = (Config::TargetMethod)cfg.target_method;
        wallfollowing_params.use_voxel = cfg.use_voxel;
        wallfollowing_params.safety_wall_distance = cfg.safety_wall_distance;
        wallfollowing_params.max_predicted_distance = cfg.max_predicted_distance;
        wallfollowing_params.emergency_slowdown = cfg.emergency_slowdown;
        wallfollowing_params.max_speed = cfg.max_speed;

        steering_params.min_possible_steering_angle = cfg.min_possible_steering_angle;
        steering_params.max_steering_angle = cfg.max_steering_angle;

        pid_params.p = cfg.pid_p;
        pid_params.i = cfg.pid_i;
        pid_params.d = cfg.pid_d;
        pid_params.anti_windup = cfg.anti_windup;
    });
}

Point Wallfollowing::determinePredictedCarPosition(ProcessedTrack& processedTrack)
{
    double prediction_distance =
        std::min(0.1 + m_speed_controller.getLastDeterminedSpeed() * 0.35, wallfollowing_params.max_predicted_distance);
    return Point{ 0, prediction_distance };
}

Point Wallfollowing::determineTrackCenter(ProcessedTrack& processed_track, Point& predicted_position)
{
    Point left_point = processed_track.left_circle.getClosestPoint(predicted_position);
    Point right_point = processed_track.right_circle.getClosestPoint(predicted_position);
    return Point{ (left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2 };
}

bool Wallfollowing::lineTooCloseToPointcloud(ProcessedTrack& processed_track, Line& line,
                                             std::vector<Point>& pointcloud)
{
    double line_length = line.length();
    for (auto& point : pointcloud)
    {
        if (GeometricFunctions::distance(processed_track.car_position, point) < line_length // +SAFETY_DISTANCE
            && GeometricFunctions::calcShortestDistanceToLine(point, line) < wallfollowing_params.safety_wall_distance)
        {
            return true;
        }
    }
    return false;
}

std::pair<Point, Point> Wallfollowing::determineTargetPathPoint(ProcessedTrack& processed_track, double min_distance,
                                                                double max_distance, double epsilon)
{
    Point predicted_position;
    Point center_point;
    bool too_close = false;
    double distance = max_distance - min_distance;
    double start = min_distance;
    double end = max_distance;
    while (distance > epsilon)
    {
        predicted_position = Point{ 0, start + distance / 2 };
        center_point = determineTrackCenter(processed_track, predicted_position);
        Line line = { processed_track.car_position, center_point };

        too_close = lineTooCloseToPointcloud(processed_track, line, processed_track.right_wall) ||
            lineTooCloseToPointcloud(processed_track, line, processed_track.left_wall) ||
            lineTooCloseToPointcloud(processed_track, line, processed_track.upper_wall);

        distance = end - start;
        if (too_close)
        {
            end = distance / 2 + start;
        }
        else
        {
            start = distance / 2 + start;
        }
    }
    return std::pair<Point, Point>(predicted_position, center_point);
}

Point Wallfollowing::determineTargetCarPosition(ProcessedTrack& processed_track, Point& predicted_position)
{
    Point left_point = processed_track.left_circle.getClosestPoint(predicted_position);
    Point right_point = processed_track.right_circle.getClosestPoint(predicted_position);
    Point central_point = Point{ (left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2 };
    double track_width = std::abs(left_point.x - right_point.x);
    Point left_car_position = Point{ processed_track.car_position.x, processed_track.car_position.y - track_width / 2 };
    Point right_car_position =
        Point{ processed_track.car_position.x, processed_track.car_position.y + track_width / 2 };

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

    if (wallfollowing_params.target_method == Config::CIRCLE_TANGENTS)
    {
        std::vector<Point> left_tangent_points;
        std::vector<Point> right_tangent_points;
        double left_center_distance =
            GeometricFunctions::distance(processed_track.car_position, processed_track.left_circle.getCenter());
        double right_center_distance =
            GeometricFunctions::distance(processed_track.car_position, processed_track.right_circle.getCenter());
        // left curve
        if (processed_track.left_circle.getCenter().x < 0 && processed_track.right_circle.getCenter().x < 0 &&
            (processed_track.left_circle.getRadius() < 1000 || processed_track.right_circle.getRadius() < 1000))
        {
            Circle safety_circle =
                Circle(processed_track.left_circle.getCenter(),
                       processed_track.left_circle.getRadius() + wallfollowing_params.safety_wall_distance);
            left_tangent_points = safety_circle.calcTangents(processed_track.car_position);
            if (left_tangent_points.size() > 0)
            {
                target_position = left_tangent_points.front();
                // target_position = Point{target_position.x + track_width / 2 -
                // wallfollowing_params.safety_wall_distance, target_position.y};
            }
        }
        // right curve
        if (processed_track.left_circle.getCenter().x > 0 and processed_track.right_circle.getCenter().x > 0 &&
            (processed_track.left_circle.getRadius() < 1000 || processed_track.right_circle.getRadius() < 1000))
        {
            Circle safety_circle =
                Circle(processed_track.right_circle.getCenter(),
                       processed_track.right_circle.getRadius() + wallfollowing_params.safety_wall_distance);
            right_tangent_points = safety_circle.calcTangents(processed_track.car_position);
            if (right_tangent_points.size() > 0)
            {
                target_position = right_tangent_points.front();
                // target_position = Point{target_position.x - track_width / 2 +
                // wallfollowing_params.safety_wall_distance, target_position.y};
            }
        }

        if (left_tangent_points.size() > 0)
        {
            std::vector<Point> rviz_tangent_points = { processed_track.car_position, target_position };
            m_rviz_geometry.showLineInRviz(20, rviz_tangent_points, ColorRGBA{ 1, 1, 1, 1.0 });
        }
        else
        {
            m_rviz_geometry.deleteMarker(20);
        }
        if (right_tangent_points.size() > 0)
        {
            std::vector<Point> rviz_tangent_points = { processed_track.car_position, target_position };
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

    Point predicted_position;
    Point target_position;
    if (wallfollowing_params.target_method == Config::CIRCLE_TANGENTS ||
        wallfollowing_params.target_method == Config::TRACK_CENTER)
    {
        predicted_position = determinePredictedCarPosition(processed_track);
        target_position = determineTargetCarPosition(processed_track, predicted_position);
    }
    else if (wallfollowing_params.target_method == Config::CENTER_PATH)
    {
        double radius = std::min(processed_track.right_circle.getRadius(), processed_track.left_circle.getRadius());
        double max_view_dist = std::max(1.0, radius / 10 * 3);
        max_view_dist = std::min(max_view_dist, 5.0);
        std::pair<Point, Point> predicted_target_position =
            determineTargetPathPoint(processed_track, 0.3, max_view_dist, 0.01);
        predicted_position = predicted_target_position.first;
        target_position = predicted_target_position.second;
        if (predicted_position.y < m_view_dist_average)
        {
            // m_view_dist_average = (predicted_position.y + m_view_dist_average * std::max(2, (int)(10 / radius))) /
            // (std::max(2, (int)(10 / radius)) + 1);
        }
        else
        {
            // m_view_dist_average = (predicted_position.y + m_view_dist_average * std::max(2, (int)(10 / radius))) /
            // (std::max(2, (int)(10 / radius)) + 1);
        }
        m_view_dist_average = (predicted_position.y + m_view_dist_average * 5) / 6;
        predicted_position.y = m_view_dist_average;
        target_position = determineTrackCenter(processed_track, predicted_position);
    }

    double angle = m_steering_controller.determineSteeringAngle(processed_track, pid_params, steering_params,
                                                                predicted_position, target_position, delta_time);

    std::vector<Point> rviz_predicted_car_points = { processed_track.car_position, predicted_position };
    m_rviz_geometry.showLineInRviz(3, rviz_predicted_car_points, ColorRGBA{ 1, 1, 1, 0.3 }, 0.005);
    std::vector<Point> rviz_target_points = { predicted_position, target_position };
    m_rviz_geometry.showLineInRviz(4, rviz_target_points, ColorRGBA{ 1, 0.4, 0, 1 });

    double predicted_distance = GeometricFunctions::distance(processed_track.car_position, predicted_position);
    double target_distance = GeometricFunctions::distance(processed_track.car_position, target_position);

    // if a curve is unexpected steep, the speed may be reduced
    if (wallfollowing_params.emergency_slowdown)
    {
        double emergency_slowdown =
            std::min(1.0, (target_distance * target_distance) / (predicted_distance * predicted_distance));
        if (emergency_slowdown > 0.8)
        {
            emergency_slowdown = 1.0;
        }
        speed *= emergency_slowdown;
    }
    speed = std::max(1.5, speed);
    speed = std::min(speed, wallfollowing_params.max_speed);

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

void Wallfollowing::handleWallsPointcloud(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wall_pointcloud,
                                          double delta_time)
{
    ProcessedTrack processed_track;
    if (m_lidar_pointcloud && m_process_track.processTrack(&processed_track, wall_pointcloud, m_lidar_pointcloud))
    {
        followWalls(processed_track, delta_time);
    }
    else
    {
        std::cerr << "processed_track not valid, falling back to laser" << std::endl;
        handleLaserPointcloud(m_laser_pointcloud, m_laser_delta_time);
    }
}

void Wallfollowing::getScanAsCartesian(std::vector<Point>* storage, const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    int n = laserscan->ranges.size();
    double skip_angle_range = ((laserscan->angle_max - laserscan->angle_min) -
                               GeometricFunctions::toRadians(wallfollowing_params.usable_laser_range)) /
        2;
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
        m_laser_delta_time = scan_time - m_last_scan_time;
        // std::cout << delta_time << std::endl;
        m_laser_pointcloud.clear();
        getScanAsCartesian(&m_laser_pointcloud, laserscan);
        if (!wallfollowing_params.use_voxel)
        {
            handleLaserPointcloud(m_laser_pointcloud, m_laser_delta_time);
        }
        double t_end = ros::Time::now().toSec();
        // std::cout << "time since last scan: " << delta_time << "s scan execution time: " << t_end - t_start << "s"
        // <<
        // std::endl;
    }
    m_last_scan_time = scan_time;
}

void Wallfollowing::wallsCallback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& walls)
{
    if (wallfollowing_params.use_voxel)
    {
        double scan_time = ros::Time::now().toSec();
        double t_start = scan_time;
        if (std::abs(scan_time - m_last_scan_time) > 0.0001 && scan_time > m_last_scan_time)
        {
            double delta_time = scan_time - m_last_scan_time;
            handleWallsPointcloud(walls, delta_time);
            double t_end = ros::Time::now().toSec();
        }
        m_last_scan_time = scan_time;
    }
}
void Wallfollowing::lidarCartesianCallback(const sensor_msgs::PointCloud2::ConstPtr& pointCloud)
{
    if (wallfollowing_params.use_voxel)
    {
        // Container for original & filtered data
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*pointCloud, *(inputCloud));

        this->m_lidar_pointcloud = inputCloud;
    }
}

void Wallfollowing::voxelCallback(const sensor_msgs::PointCloud2::ConstPtr& voxel_msg)
{
    // double scan_time = ros::Time::now().toSec();
    // double t_start = ros::Time::now().toSec();
    // if (std::abs(scan_time - m_last_scan_time) > 0.0001 && scan_time > m_last_scan_time)
    // {
    //     double delta_time = scan_time - m_last_scan_time;

    //     std::vector<Point> pointcloud;

    //     for (size_t i = 0; i < voxel_msg->width; i++)
    //     {
    //         Point p;
    //         uint32_t tmp_score;
    //         memcpy(&p.y, &voxel_msg->data[i * voxel_msg->point_step + voxel_msg->fields[0].offset], sizeof(float));
    //         memcpy(&p.x, &voxel_msg->data[i * voxel_msg->point_step + voxel_msg->fields[1].offset], sizeof(float));
    //         p.x = -p.x;
    //         if (!std::isnan(p.x) && !std::isinf(p.x) && !std::isnan(p.y) && !std::isinf(p.y))
    //         {
    //             pointcloud.push_back(p);
    //             // std::cout << "x: " << p.x << " y: " << p.y << std::endl;
    //         }
    //     }

    //     handleLaserPointcloud(pointcloud, delta_time);
    //     double t_end = ros::Time::now().toSec();
    //     // std::cout << "time since last scan: " << delta_time << "s scan execution time: " << t_end - t_start << "s"
    //     <<
    //     // std::endl;
    // }
    // m_last_scan_time = scan_time;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wallfollowing");
    Wallfollowing wallfollowing;
    ros::spin();
    return EXIT_SUCCESS;
}
