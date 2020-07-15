#include "log_stats.h"

LogStats::LogStats()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &LogStats::controlledDriveParametersCallback, this);
    m_max_speed_subscriber =
        m_node_handle.subscribe<std_msgs::Float64>(TOPIC_MAX_SPEED, 1, &LogStats::maxSpeedCallback, this);
    m_gazebo_state_telemetry_subscriber =
        m_node_handle.subscribe<drive_msgs::gazebo_state_telemetry>(TOPIC_GAZEBO_STATE_TELEMETRY, 1,
                                                                    &LogStats::gazeboStateTelemetryCallback, this);

    m_hud_speed_publisher = m_node_handle.advertise<std_msgs::Float32>("hud_speed_value", 1);
}

void LogStats::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_current_speed = parameters->velocity;
    m_current_steering_angle = parameters->angle;

    // hud publisher example
    std_msgs::Float32 speed_message;
    speed_message.data = m_current_speed;
    m_hud_speed_publisher.publish(speed_message);
}

void LogStats::maxSpeedCallback(const std_msgs::Float64::ConstPtr& max_speed)
{
    m_max_speed = max_speed->data;
}

void LogStats::gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr& gazebo_state_telemetry)
{
    m_gazebo_wheel_speed = gazebo_state_telemetry->wheel_speed;
    m_gazebo_car_speed = gazebo_state_telemetry->car_speed;
    // ... usw
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "log_stats");
    LogStats log_stats;
    ros::spin();
    return EXIT_SUCCESS;
}