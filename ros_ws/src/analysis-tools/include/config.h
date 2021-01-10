#pragma once

namespace Config
{
    static const float USABLE_LASER_RANGE = 220;
    static const double MAX_STEERING_ANGLE = 40.0f;
    static const double RPM_FACTOR = 1299.224f;

    // timeinterval in seconds for writing line to logfile
    static const double LOG_INTERVAL = 0.05f;

    // float precision on writing to hud publishers
    static const double HUD_PRECISION = 4U;

    // time in intervals to wait for writing values to logfile
    static const unsigned int LOGENTRY_OFFSET = 100U;
} // namespace Config