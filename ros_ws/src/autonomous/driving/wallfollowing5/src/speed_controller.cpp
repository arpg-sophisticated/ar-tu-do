#include "speed_controller.h"

using namespace std;

SpeedController::SpeedController()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &SpeedController::controlledDriveParametersCallback, this);
}

double SpeedController::calcMaxCurveSpeed(double radius)
{
    return sqrt(PhysicalProperties::DYNAMIC_FRICTION * 9.81 * radius);
}

double SpeedController::calcMaxSpeed(double distance, double target_speed)
{
    return sqrt((2 * distance * PhysicalProperties::ACCELERATION * PhysicalProperties::ACCELERATION +
                 m_current_speed * m_current_speed * PhysicalProperties::ACCELERATION +
                 target_speed * target_speed * PhysicalProperties::ACCELERATION) /
                (PhysicalProperties::ACCELERATION + PhysicalProperties::ACCELERATION));
}

double SpeedController::calcBrakingDistance(double distance, double target_speed)
{
    return (2 * distance * PhysicalProperties::ACCELERATION + m_current_speed * m_current_speed -
            target_speed * target_speed) /
        (2 * PhysicalProperties::ACCELERATION + 2 * PhysicalProperties::ACCELERATION);
}

double SpeedController::calcSpeed(ProcessedTrack& processed_track)
{
    double remaining_distance = processed_track.curve_entry.y;
    double radius = min(processed_track.left_circle.getRadius(), processed_track.right_circle.getRadius());
    double speed = calcMaxCurveSpeed(radius);
    if (processed_track.curve_type != CURVE_TYPE_STRAIGHT)
    {
        double safety_margin = 0.25;
        if (remaining_distance < 5)
        {
            safety_margin = 0.05 * remaining_distance;
        }
        double target_speed = calcMaxCurveSpeed(processed_track.upper_circle.getRadius()) * 0.9;
        double braking_distance = calcBrakingDistance(remaining_distance, target_speed) + safety_margin;
        if (remaining_distance > braking_distance)
        {
            speed = min(calcMaxSpeed(remaining_distance, target_speed), speed);
        }
        else
        {
            speed = target_speed;
        }
    }
    // speed *= std::min(1.0, radius / 3.0);
    m_last_determined_speed = speed;
    return speed;
}

void SpeedController::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_current_speed = parameters->velocity;
}