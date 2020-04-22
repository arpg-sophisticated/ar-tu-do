#include "speed_controller.h"

using namespace std;

SpeedController::SpeedController()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &SpeedController::controlledDriveParametersCallback, this);
}

double SpeedController::convertRpmToSpeed(double rpm)
{
    // 1299.224 is the conversion factor from electrical revolutions per minute to m/s
    // and is derived from the transmission and the rotational speed of the motor.
    // More values describing the car properties can be found in car_config.h.
    return rpm / 1299.224;
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

double SpeedController::calcSpeed(Circle* left_circle, Circle* right_circle, Circle* upper_circle,
                                  double remaining_distance)
{
    double radius = min(left_circle->getRadius(), right_circle->getRadius());
    double speed = calcMaxCurveSpeed(radius);
    if (remaining_distance >= 0 && upper_circle != nullptr)
    {
        double safety_margin = 0.25;
        if (remaining_distance < 5)
        {
            safety_margin = 0.05 * remaining_distance;
        }
        double target_speed = calcMaxCurveSpeed(upper_circle->getRadius());
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
    return speed;
}

void SpeedController::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_current_speed = parameters->velocity;
}