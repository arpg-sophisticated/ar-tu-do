#pragma once

#include <ros/ros.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic pop

#include <sensor_msgs/Imu.h>

constexpr const char* TOPIC_IMU_DATA = "/imu";
constexpr const char* TOPIC_VELOCITY_OUTPUT = "/racer/velocity";
constexpr const char* MODEL_BASE_LINK = "base_link";

/**
 */
class ImuAccumulator
{
    public:
    ImuAccumulator();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_imu_subscriber;
    ros::Publisher m_speed_publisher;

    double xAcc;
    double yAcc;
    double zAcc;

    double initX;
    double initY;
    double initZ;
    bool initSet;

    ros::Time lastValueTime;

    void scanCallback(const sensor_msgs::Imu::ConstPtr& imu);
};