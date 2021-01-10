#pragma once

#include "car_config.h"

namespace Config
{
    enum TargetMethod
    {
        TRACK_CENTER,
        CIRCLE_TANGENTS,
        CENTER_PATH
    };

    // safety margin in m the car brakes before a curve entry
    static const float SAFETY_MARGIN = 0.25;
    // The size of the part of the laser scan that should be used by the algorithm, in degrees.
    static const float USABLE_LASER_RANGE = 240;

    static const TargetMethod target_method = CIRCLE_TANGENTS;

    static const float SAFETY_WALL_DISTANCE = 1.2 * car_config::REAR_WHEEL_DISTANCE;

    static const double MAX_PREDICTED_DISTANCE = 2.2;

    static const bool EMERGENCY_SLOWDOWN = true;
} // namespace Config

namespace PID
{
    static const double P = 4;
    static const double I = 0.2;
    static const double D = 0.02;
    static const double ANTI_WINDUP = 0.2;
} // namespace PID