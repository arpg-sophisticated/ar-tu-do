#include "steering_controller.h"

double SteeringController::determineSteeringAngle(Point& car_position, Point& predicted_position,
                                                  Point& target_position, double delta_time)
{
    double distance_to_target = GeometricFunctions::distance(car_position, target_position);
    double error = (target_position.x - predicted_position.x) / distance_to_target;
    double steering_angle = m_pid_controller.updateAndGetCorrection(error, delta_time);
    return steering_angle;
}

void SteeringController::showSteeringAngle()
{
}