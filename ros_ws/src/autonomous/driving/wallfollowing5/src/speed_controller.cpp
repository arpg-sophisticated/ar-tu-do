#include "speed_controller.h"

using namespace std;

SpeedController::SpeedController()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &SpeedController::controlledDriveParametersCallback, this);
}

double SpeedController::calcMaxCurveSpeed(double radius, double friction)
{
    return sqrt(friction * 9.81 * radius);
}

double SpeedController::calcMaxSpeed(double distance, double target_speed, double acceleration)
{
    return sqrt((2 * distance * acceleration * acceleration + m_current_speed * m_current_speed * acceleration +
                 target_speed * target_speed * acceleration) /
                (acceleration + acceleration));
}

double SpeedController::calcBrakingDistance(double distance, double target_speed, double acceleration)
{
    return (2 * distance * acceleration + m_current_speed * m_current_speed - target_speed * target_speed) /
        (2 * acceleration + 2 * acceleration);
}

double SpeedController::calcSpeed(ProcessedTrack& processed_track, Config::SpeedParams& speed_config,
                                  double acceleration, double dynamic_friction)
{
    double remaining_distance = processed_track.curve_entry.y;
    double radius = min(processed_track.left_circle.getRadius(), processed_track.right_circle.getRadius());
    double speed = calcMaxCurveSpeed(radius, dynamic_friction);
    if (processed_track.curve_type != CURVE_TYPE_STRAIGHT)
    {
        // double safety_margin = 0.25;
        // if (remaining_distance < 5)
        // {
        //     safety_margin = 0.05 * remaining_distance;
        // }
        double target_speed = calcMaxCurveSpeed(processed_track.upper_circle.getRadius(), dynamic_friction) * 0.9;
        double braking_distance =
            calcBrakingDistance(remaining_distance, target_speed, acceleration) + speed_config.brake_safety_margin;
        if (remaining_distance > braking_distance)
        {
            speed = min(calcMaxSpeed(remaining_distance, target_speed, acceleration), speed);
        }
        else
        {
            speed = target_speed;
        }
    }
    m_last_determined_speed = speed;
    return speed;
}

void SpeedController::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_current_speed = parameters->velocity;
}