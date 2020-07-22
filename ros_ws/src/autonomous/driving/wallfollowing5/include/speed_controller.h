#pragma once

#include "circle.h"
#include "config.h"
#include "physical_properties.h"
#include "process_track.h"
#include <cmath>
#include <drive_msgs/drive_param.h>
#include <ros/ros.h>

class SpeedController
{
    private:
    double m_current_speed = 0;
    double m_last_determined_speed = 0;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;

    public:
    SpeedController();

    double getCurrentSpeed()
    {
        return m_current_speed;
    }
    double getLastDeterminedSpeed()
    {
        return m_last_determined_speed;
    }
    double calcMaxCurveSpeed(double radius);
    double calcMaxSpeed(double distance, double target_speed);
    double calcBrakingDistance(double distance, double target_speed);
    double calcSpeed(ProcessedTrack& processed_track);

    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};