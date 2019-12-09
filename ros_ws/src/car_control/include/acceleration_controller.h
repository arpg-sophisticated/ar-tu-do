#pragma once
#include <ros/ros.h>
#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <car_config.h>

const static float INITIAL_DYNAMIC_FRICTION = 0.5;
const static float INITIAL_STATIC_FRICTION = 0.75;
const static float CAR_ACCELERATION = 9.81 * INITIAL_DYNAMIC_FRICTION;

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

    float getDynamicFriction() {
        float friction;
        return m_node_handle.getParam("/physical_properties/dynamic_friction", friction) ? friction : 0;
    }

    float getStaticFriction() {
        float friction;
        return m_node_handle.getParam("/physical_properties/static_friction", friction) ? friction : 0;
    }

    void approachSpeedControlled(const ros::TimerEvent& event);

    float calcRadiusFromAngle(float angle);
    float calcMaxSpeed(float angle, float friction);
    void setControlledTargetSpeed(float parameters_angle, float parameters_velocity);
    /**
     * @brief deals with incomming drive param messages
     */
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    void brakeCallback(const std_msgs::Float64::ConstPtr& message);
};
