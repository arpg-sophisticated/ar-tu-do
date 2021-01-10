#include "steering_controller.h"

SteeringController::SteeringController()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &SteeringController::controlledDriveParametersCallback, this);
}

double SteeringController::determineSteeringAngle(ProcessedTrack& processed_track, Config::PIDParams& pid_params,
                                                  Config::SteeringParams& steering_params, Point& predicted_position,
                                                  Point& target_position, double acceleration, double delta_time)
{
    double distance_to_target = GeometricFunctions::distance(processed_track.car_position, target_position);
    double error = (target_position.x - predicted_position.x) / distance_to_target;
    double steering_angle = m_pid_controller.updateAndGetCorrection(error, delta_time, pid_params);
    double min_turning_radius = std::pow(m_current_speed + 0.01, 2) / acceleration;
    min_turning_radius = std::max(car_config::WHEELBASE + 0.01, min_turning_radius);

    double predicted_steering_angle =
        std::atan(car_config::WHEELBASE /
                  std::sqrt((std::pow(min_turning_radius, 2) - std::pow(car_config::WHEELBASE / 2, 2)))) /
        (steering_params.max_steering_angle * (GeometricFunctions::PI / 180));

    double min_steering_angle = std::max(-0.99, -predicted_steering_angle * 1.2);
    min_steering_angle = std::min(min_steering_angle, -steering_params.min_possible_steering_angle);
    double max_steering_angle = std::min(predicted_steering_angle * 1.2, 0.99);
    max_steering_angle = std::max(steering_params.min_possible_steering_angle, max_steering_angle);
    double old_steering_angle = steering_angle;

    steering_angle = std::max(min_steering_angle, steering_angle);
    steering_angle = std::min(steering_angle, max_steering_angle);
    return steering_angle;
}

void SteeringController::showSteeringAngle()
{
}

void SteeringController::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_current_speed = parameters->velocity;
}