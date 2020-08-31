#include "car_controller.h"
// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

#include <boost/algorithm/clamp.hpp>

CarController::CarController()
    : m_drive_param_lock{ true }
    , m_emergency_stop_lock{ true }
{
    this->m_drive_parameters_subscriber =
        this->m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                               &CarController::driveParametersCallback, this);
    this->m_drive_mode_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int32>(TOPIC_DRIVE_MODE, 1, &CarController::driveModeCallback, this);
    this->m_emergency_stop_subscriber =
        this->m_node_handle.subscribe<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1, &CarController::emergencyStopCallback,
                                                      this);

    this->m_speed_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_angle_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_ANGLE, 1);
    this->m_brake_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_BRAKE, 1);
    // Make a publisher for drive messages
    drive_pub = m_node_handle.advertise<ackermann_msgs::AckermannDriveStamped>(DRIVE_TOPIC, 10);

    // get car parameters
    m_node_handle.getParam("max_speed", m_max_speed);
    m_node_handle.getParam("max_steering_angle", m_max_steering_angle);
}

void CarController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    this->publishDriveParameters((m_drive_param_lock || m_emergency_stop_lock) ? 0 : parameters->velocity,
                                 m_drive_param_lock ? 0 : parameters->angle);
}

void CarController::publishDriveParameters(float speed, float relative_angle)
{
    float rpm = convertSpeedToRpm(speed);
    float angle = (relative_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2;

    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed;
    drive_msg.steering_angle = relative_angle * -0.4189; // m_max_steering_angle doesn't work
    drive_st_msg.drive = drive_msg;
    drive_pub.publish(drive_st_msg);

    this->publishSpeed(rpm);
    this->publishAngle(angle);

    ROS_DEBUG_STREAM("running: "
                     << " | speed: " << speed << " | angle: " << angle);
}

int CarController::convertSpeedToRpm(float speed)
{
    // 0.9 is an experimental derived correction factor for the speed
    return (speed / 0.9) * car_config::TRANSMISSION / car_config::ERPM_TO_SPEED;
}

void CarController::publishSpeed(float speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_speed_publisher.publish(speed_message);
}

void CarController::publishAngle(float angle)
{
    std_msgs::Float64 angle_message;
    angle_message.data = angle;
    this->m_angle_publisher.publish(angle_message);
}

void CarController::driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message)
{
    this->m_current_drive_mode = (DriveMode)drive_mode_message->data;
    this->m_drive_param_lock = this->m_current_drive_mode == DriveMode::LOCKED;
    if (this->m_drive_param_lock)
        this->stop();
}

void CarController::emergencyStopCallback(const std_msgs::Bool::ConstPtr& emergency_stop_message)
{
    bool enable_emergency_stop = emergency_stop_message->data && this->m_current_drive_mode != DriveMode::MANUAL;
    this->m_emergency_stop_lock = enable_emergency_stop;
    if (this->m_emergency_stop_lock)
        this->stop();
}

void CarController::stop()
{
    this->publishSpeed(0);

    std_msgs::Float64 brake_message;
    brake_message.data = 4;
    this->m_brake_publisher.publish(brake_message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    CarController carController;
    ros::spin();
    return EXIT_SUCCESS;
}
