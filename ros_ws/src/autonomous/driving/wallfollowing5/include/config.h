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
        float usable_laser_range = 240;

        TargetMethod target_method = TRACK_CENTER;

        bool use_voxel = false;

        float safety_wall_distance = 1.2 * car_config::REAR_WHEEL_DISTANCE;

        double max_predicted_distance = 1.0;

        bool emergency_slowdown = true;

        double max_speed = 2.5;
    };

    // steering-parameters
    struct SteeringParams
    {
        double min_possible_steering_angle;
        double max_steering_angle;
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