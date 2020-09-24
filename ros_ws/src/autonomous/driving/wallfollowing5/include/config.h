#pragma once

#include "car_config.h"
#include <dynamic_reconfigure/server.h>
#include <wallfollowing5/wallfollowing5Config.h>

constexpr const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";

namespace Config
{
    enum TargetMethod
    {
        TRACK_CENTER,
        CIRCLE_TANGENTS,
        CENTER_PATH
    };

    // wallfollowing-parameters
    struct WallfollowingParams
    {
        // The size of the part of the laser scan that should be used by the algorithm, in degrees.
        double usable_laser_range;
        double max_laser_range;

        TargetMethod target_method;

        bool use_voxel;

        bool use_obstacle_avoidence;

        bool use_imaginary_track_center = false;

        double safety_wall_distance;

        double max_predicted_distance;

        bool emergency_slowdown;

        double advanced_trajectory;
        double advanced_trajectory_distance;

        double max_speed;
    };

    struct ProcessingParams
    {
        double usable_laser_range;
        double usable_laser_range_wall_detection;
        double radius_curve_entry_proportion;
    };

    // steering-parameters
    struct SteeringParams
    {
        double min_possible_steering_angle;
        double max_steering_angle;
    };

    struct SpeedParams
    {
        double brake_safety_margin;
    };

    // pid-parameters
    struct PIDParams
    {
        double p;
        double i;
        double d;
        double anti_windup;
    };
}; // namespace Config