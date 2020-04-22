#pragma once

#include "circle.h"
#include "config.h"
#include "physical_properties.h"
#include <cmath>
#include <drive_msgs/drive_param.h>
#include <ros/ros.h>

constexpr const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";

class SpeedController
{
    private:
    double m_current_speed;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;

    public:
    SpeedController();

    double getCurrentSpeed()
    {
        return m_current_speed;
    }
    double convertRpmToSpeed(double rpm);
    double calcMaxCurveSpeed(double radius);
    double calcMaxSpeed(double distance, double target_speed);
    double calcBrakingDistance(double distance, double target_speed);
    double calcSpeed(Circle* left_circle, Circle* right_circle, Circle* upper_circle, double remaining_distance);

    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};