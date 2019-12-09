#pragma once

#include "car_config.h"
#include "drive_mode.h"
#include <ros/ros.h>

#include <algorithm>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";
constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";
constexpr const char* TOPIC_EMERGENCY_STOP = "/commands/emergency_stop";

class CarController
{
    public:
    CarController();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_drive_parameters_subscriber;
    ros::Subscriber m_drive_mode_subscriber;
    ros::Subscriber m_emergency_stop_subscriber;

    ros::Publisher m_speed_publisher;
    ros::Publisher m_angle_publisher;
    ros::Publisher m_brake_publisher;

    bool m_drive_param_lock;
    bool m_emergency_stop_lock;
    DriveMode m_current_drive_mode;

    /**
     * @brief deals with incomming drive param messages
     */
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    /**
     * @brief sets the current drive mode
     */
    void driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message);

    /**
     * @brief callback for the topic that enables / disables the motor
     */
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& drive_mode_message);

    /**
     * @brief takes a speed and angle, converts and forwards them to gazebo/focbox
     * @param raw_speed the speed in m/s
     */
    void publishDriveParameters(float raw_speed, float raw_angle);

    /**
     * @brief takes speed and publishes it to gazebo/focbox
     * @param speed the speed in m/s which is converted to rpm and then sent to the focbox
     */
    void publishSpeed(float speed);

    /**
     * @brief takes angle and publishes it to gazebo/focbox
     */
    void publishAngle(float angle);

    /**
     * @brief publishes a brake message that stops the car
     */
    void stop();

    /**
     * @brief converts m/s in revolutions per minute
     * @param speed speed in m/s
     * @return revolutions per minute
     */
    int convertSpeedToRpm(float speed)
    {
        return speed * car_config::TRANSMISSION / car_config::ERPM_TO_SPEED;
    }
};
