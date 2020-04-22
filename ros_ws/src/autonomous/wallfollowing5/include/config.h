#pragma once

namespace Config
{
    // safety margin in m the car brakes before a curve entry
    static const float SAFETY_MARGIN = 0.25;
    // The size of the part of the laser scan that should be used by the algorithm, in degrees.
    static const float USABLE_LASER_RANGE = 220;
}

namespace PID
{
    static const double P = 4;
    static const double I = 0.2;
    static const double D = 0.02;
}