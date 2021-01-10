#include "acceleration_controller.h"

AccelerationController::AccelerationController()
{
    m_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                         &AccelerationController::driveParametersCallback, this);
    m_brake_subscriber =
        m_node_handle.subscribe<std_msgs::Float64>(TOPIC_FOCBOX_BRAKE, 1, &AccelerationController::brakeCallback, this);

    m_controlled_drive_parameters_publisher =
        m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1);
    m_emergency_stop_publisher = m_node_handle.advertise<std_msgs::Time>(TOPIC_EMERGENCY_STOP, 1);

    m_timer = m_node_handle.createTimer(ros::Duration(0.01), &AccelerationController::approachSpeedControlled, this);
}

void AccelerationController::approachSpeedControlled(const ros::TimerEvent& event)
{
    ros::Duration duration = event.current_real - event.last_real;
    double diff_speed = m_target_speed - m_current_speed;
    double delta_speed;
    if (diff_speed >= 0)
    {
        delta_speed = PhysicalProperties::getAcceleration() * duration.toSec();
    }
    else
    {
        delta_speed = -PhysicalProperties::getAcceleration() * duration.toSec();
    }

    if (std::abs(delta_speed) > std::abs(diff_speed))
    {
        m_current_speed = m_target_speed;
    }
    else
    {
        m_current_speed += delta_speed;
    }

    if (m_current_speed > 0)
    {
        m_current_speed = std::max(0.5, m_current_speed);
    }

    drive_msgs::drive_param message;
    message.velocity = m_current_speed;
    message.angle = m_angle;
    m_controlled_drive_parameters_publisher.publish(message);
}

void AccelerationController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_target_speed = parameters->velocity;
    m_angle = parameters->angle;
}

void AccelerationController::brakeCallback(const std_msgs::Float64::ConstPtr& message)
{
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
