#pragma once

#include "config.h"
#include <ctime>
#include <deque>
#include <drive_msgs/drive_param.h>
#include <drive_msgs/gazebo_state_telemetry.h>
#include <fstream>
#include <iostream>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sys/stat.h>

constexpr const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";
constexpr const char* TOPIC_MAX_SPEED = "/speed_info/max_speed";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";

class BruteforceReward
{
    private:
    ros::Subscriber m_controlled_drive_parameters_subscriber;
    ros::Subscriber m_max_speed_subscriber;
    ros::Subscriber m_gazebo_state_telemetry_subscriber;
    ros::NodeHandle m_node_handle;

    // counter for logentry
    unsigned int m_logentry;

    // current time
    ros::Time m_time_start;
    ros::Duration m_time_last;
    ros::Duration m_time_delta;
    ros::Duration m_time_interval;
    ros::Duration m_time_current;
    
    // several values
    double m_speed_current;
    std::deque<double> m_speed_current_average;
    int m_average;
    std::string m_resultsfile;
    std::string m_command;

    public:
    BruteforceReward();

    private:
    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& drive_parameters);
    void maxSpeedCallback(const std_msgs::Float64::ConstPtr& max_speed);
};
