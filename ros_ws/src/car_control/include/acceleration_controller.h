#pragma once
#include <car_config.h>
#include <drive_msgs/drive_param.h>
#include <physical_properties.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>

const static bool EMERGENCY_STOP_ACTIVE = false;

constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";

constexpr const char* TOPIC_EMERGENCY_STOP = "/input/emergencystop";

class AccelerationController
{
    public:
    AccelerationController();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_drive_parameters_subscriber;
    ros::Subscriber m_brake_subscriber;

    ros::Publisher m_controlled_drive_parameters_publisher;
    ros::Publisher m_emergency_stop_publisher;

    ros::Timer m_timer;

    double m_current_speed = 0;
    double m_target_speed = 0;

    double m_angle = 0;

    /**
     * Approaches m_target_speed over time with a certain acceleration given with CAR_ACCELERATION
     **/
    void approachSpeedControlled(const ros::TimerEvent& event);

    /**
     * @brief deals with incomming drive param messages
     */
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    void brakeCallback(const std_msgs::Float64::ConstPtr& message);
};
