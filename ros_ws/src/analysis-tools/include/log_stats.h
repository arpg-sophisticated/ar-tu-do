#pragma once

#include <drive_msgs/drive_param.h>
#include <ros/ros.h>

constexpr const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";

class LogStats
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;

    double m_current_speed;

    public:
    LogStats();

    private:
    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};