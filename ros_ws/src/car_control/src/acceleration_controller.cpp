#include "acceleration_controller.h"

AccelerationController::AccelerationController()
{
    m_drive_parameters_subscriber = m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                               &AccelerationController::driveParametersCallback, this);
    m_brake_subscriber = m_node_handle.subscribe<std_msgs::Float64>(TOPIC_FOCBOX_BRAKE, 1, &AccelerationController::brakeCallback, this);

    m_controlled_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1);
    m_emergency_stop_publisher = m_node_handle.advertise<std_msgs::Time>(TOPIC_EMERGENCY_STOP, 1);

    m_timer = m_node_handle.createTimer(ros::Duration(0.02), &AccelerationController::approachSpeedControlled, this);

    // add physical properties to the parameter server so that they can be used in other nodes and could be changed
    m_node_handle.setParam("/physical_properties/dynamic_friction", INITIAL_DYNAMIC_FRICTION);
    m_node_handle.setParam("/physical_properties/static_friction", INITIAL_STATIC_FRICTION);
    m_node_handle.setParam("/physical_properties/acceleration", CAR_ACCELERATION);
}

void AccelerationController::approachSpeedControlled(const ros::TimerEvent& event)
{
    ros::Duration duration = event.current_real - event.last_real;
    double diff_speed = m_target_speed - m_current_speed;
    double delta_speed;
    if (diff_speed >= 0) 
    {
        delta_speed = CAR_ACCELERATION * duration.toSec();
    } else 
    {
        delta_speed = -CAR_ACCELERATION * duration.toSec();
    }
    if (std::abs(delta_speed) > std::abs(diff_speed))
    {
        m_current_speed = m_target_speed;
    } else 
    {
        m_current_speed += delta_speed;
    }
    drive_msgs::drive_param message;
    message.velocity = m_current_speed;
    message.angle = m_angle;
    m_controlled_drive_parameters_publisher.publish(message);
}

float AccelerationController::calcRadiusFromAngle(float angle) {
    float angle_rad = angle * car_config::MAX_STEERING_ANGLE;
    if (angle_rad == 0) {
        angle_rad = 0.0001;
    }
    return car_config::WHEELBASE / std::sin(angle_rad);
}

float AccelerationController::calcMaxSpeed(float angle, float friction) {
    return std::sqrt(friction * 9.81 * std::abs(calcRadiusFromAngle(angle)));
}

void AccelerationController::setControlledTargetSpeed(float parameters_angle, float parameters_velocity) {
    float friction, max_speed_dynamic_friction, max_speed_static_friction;
    max_speed_dynamic_friction = calcMaxSpeed(parameters_angle, getDynamicFriction());
    // checks if max speed for dynamic friction is lower than the current speed so that the car is in a good controllable state
    if (max_speed_dynamic_friction >= m_current_speed)  {
        // checks if the max speed for dynamic friction is lower than the target speed which otherwise can't be used
        if (max_speed_dynamic_friction >= parameters_velocity) {
            m_target_speed = parameters_velocity;
        } else {
            m_target_speed = max_speed_dynamic_friction;
            std::cerr << "Target Speed (" << parameters_velocity << " m/s) exceeds the maximal possible velocity (" << max_speed_dynamic_friction << " m/s) for dynamic friction." << std::endl;
        }
    } else {
        std::cerr << "The car could be in a hard controllable state, because the current speed (" << m_current_speed << " m/s) exceeds the maximal possible velocity (" << max_speed_dynamic_friction << " m/s) for dynamic friction" << std::endl;
        max_speed_static_friction = calcMaxSpeed(parameters_angle, getStaticFriction());
        if (max_speed_static_friction >= m_current_speed) {
            // drive parameters could lead to a hard controllable state
            if (max_speed_dynamic_friction >= parameters_velocity) {
                m_target_speed = parameters_velocity;
            } else {
                m_target_speed = max_speed_dynamic_friction;
                std::cerr << "Target Speed (" << parameters_velocity << " m/s) exceeded the maximal possible velocity (" << max_speed_dynamic_friction << " m/s) for dynamic friction." << std::endl;
            }
        } else {
            // drive parameters could lead to an unrecoverable state
            if (EMERGENCY_STOP_ACTIVE) {
                std_msgs::Time message;
                message.data = ros::Time::now();
                m_emergency_stop_publisher.publish(message);
            } else {
                m_target_speed = std::min(max_speed_dynamic_friction, parameters_velocity);
            }
            std::cerr << "The car could be in an unrecoverable state -> Emergency Stop" << std::endl;
        }
    }
}

void AccelerationController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters) {
    setControlledTargetSpeed(parameters->angle, parameters->velocity);
    m_angle = parameters->angle;
}

void AccelerationController::brakeCallback(const std_msgs::Float64::ConstPtr& message) {
    m_current_speed = 0;
    m_target_speed = 0;
    m_angle = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "acceleration_controller");
    AccelerationController accelerationController;
    ros::spin();
    return EXIT_SUCCESS;
}
