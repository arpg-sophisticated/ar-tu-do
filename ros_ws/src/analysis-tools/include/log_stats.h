#pragma once

#include <drive_msgs/drive_param.h>
#include <drive_msgs/gazebo_state_telemetry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

constexpr const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";
constexpr const char* TOPIC_MAX_SPEED = "/speed_info/max_speed";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";

class LogStats
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;
    ros::Subscriber m_max_speed_subscriber;
    ros::Subscriber m_gazebo_state_telemetry_subscriber;
    ros::Publisher m_hud_speed_publisher;

    double m_current_speed;
    double m_current_steering_angle;

    double m_max_speed;

    double m_gazebo_wheel_speed;
    double m_gazebo_car_speed;

    public:
    LogStats();

    private:
    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& drive_parameters);
    void maxSpeedCallback(const std_msgs::Float64::ConstPtr& max_speed);
    void gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr& gazebo_state_telemetry);
};