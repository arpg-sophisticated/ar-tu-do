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

    // safety margin in m the car brakes before a curve entry
    static float SAFETY_MARGIN = 0.25;
    // The size of the part of the laser scan that should be used by the algorithm, in degrees.
    static float USABLE_LASER_RANGE = 240;

    static TargetMethod TARGET_METHOD = TRACK_CENTER;

    static bool USE_VOXEL = false;

    static float SAFETY_WALL_DISTANCE = 1.2 * car_config::REAR_WHEEL_DISTANCE;

    static double MAX_PREDICTED_DISTANCE = 1.0;

    static bool EMERGENCY_SLOWDOWN = true;

    static double MAX_SPEED = 2.5;

    static double MIN_POSSIBLE_STEERING_ANGLE = 0.3;
} // namespace Config

namespace PID
{
    static double P = 4;
    static double I = 0.2;
    static double D = 0.02;
    static double ANTI_WINDUP = 0.2;
} // namespace PID

class DynamicConfig
{
    private:
    dynamic_reconfigure::Server<wallfollowing5::wallfollowing5Config> m_dyn_cfg_server;

    public:
    DynamicConfig()
    {
        m_dyn_cfg_server.setCallback([&](wallfollowing5::wallfollowing5Config& cfg, uint32_t) {
            Config::USABLE_LASER_RANGE = cfg.usable_laser_range;
            Config::TARGET_METHOD = (Config::TargetMethod)cfg.target_method;
            Config::USE_VOXEL = cfg.use_voxel;
            Config::SAFETY_WALL_DISTANCE = cfg.safety_wall_distance;
            Config::MAX_PREDICTED_DISTANCE = cfg.max_predicted_distance;
            Config::EMERGENCY_SLOWDOWN = cfg.emergency_slowdown;
            Config::MAX_SPEED = cfg.max_speed;
            Config::MIN_POSSIBLE_STEERING_ANGLE = cfg.min_possible_steering_angle;

            PID::P = cfg.pid_p;
            PID::I = cfg.pid_i;
            PID::D = cfg.pid_d;
            PID::ANTI_WINDUP = cfg.anti_windup;
        });
    }
};