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

class LogStats
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;
    ros::Subscriber m_max_speed_subscriber;
    ros::Subscriber m_gazebo_state_telemetry_subscriber;
    ros::Publisher m_hud_speed_publisher;
    ros::Publisher m_hud_maxspeed_publisher;
    ros::Publisher m_hud_rpm_publisher;
    ros::Publisher m_hud_acceleration_publisher;
    ros::Publisher m_hud_angle_publisher;
    ros::Publisher m_hud_distance_publisher;
    ros::Publisher m_hud_clock_publisher;
    ros::Publisher m_hud_text_publisher;

    // paths and filenames
    std::string m_filenamedat;
    std::string m_filenamecsv;

    // create file handlers
    std::ofstream m_filestream_dat;
    std::ofstream m_filestream_csv;

    // counter for logentry
    unsigned int m_logentry;

    // current time
    ros::Time m_time_start;
    ros::Duration m_time_last;
    ros::Duration m_time_delta;
    ros::Duration m_time_interval;
    ros::Duration m_time_current;

    // parameters
    std::string m_log_prefix;
    int m_mean_length;
    int m_smooth_count;
    bool m_is_simulation;
    bool m_write_logfile;

    // incoming data
    // double m_current_speed;
    // double m_current_steering_angle;
    // double m_max_speed;
    // double m_gazebo_wheel_speed; // ???
    // double m_gazebo_car_speed; // ???

    // calculated data - speed
    double m_speed_current;
    double m_speed_last;
    double m_speed_delta;
    double m_speed_avg;
    std::deque<double> m_speed_avgtime;
    double m_speed_max;
    std::deque<double> m_speed_maxtime;
    std::deque<double> m_speed_smooth;

    // calculated data - maximum speed from algorithm
    double m_maxspeed_value;
    double m_maxspeed_current;
    double m_maxspeed_last;
    double m_maxspeed_delta;
    double m_maxspeed_avg;
    std::deque<double> m_maxspeed_smooth;

    // calculated data - driven distance
    double m_distance_current;
    double m_distance_last;
    double m_distance_delta;

    // calculated data - acceleration
    double m_acceleration_current;
    double m_acceleration_last;
    double m_acceleration_min;
    double m_acceleration_max;
    std::deque<double> m_acceleration_mintime;
    std::deque<double> m_acceleration_maxtime;
    std::deque<double> m_acceleration_smooth;
    double m_acceleration_delta;

    // calculated data - turn (0..1)
    double m_turn_current;
    double m_turn_last;
    double m_turn_delta;

    // calculated data - angle derived from turn and MAX_STEERING_ANGLE
    double m_angle_current;
    double m_angle_last;
    double m_angle_delta;
    std::deque<double> m_angle_smooth;

    // helper
    std::string getLogPath();
    std::string getTimeString();

    // log writing
    void writeLogFile(std::string delimiter, std::string filename);
    void publishHud();

    public:
    LogStats();

    private:
    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& drive_parameters);
    void maxSpeedCallback(const std_msgs::Float64::ConstPtr& max_speed);
    // void gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr& gazebo_state_telemetry);
};