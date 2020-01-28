#include "imu_accumulator.h"
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ImuAccumulator::ImuAccumulator()
{
    m_imu_subscriber =
        m_node_handle.subscribe<sensor_msgs::Imu>(TOPIC_IMU_DATA, 100, &ImuAccumulator::scanCallback, this);
    m_speed_publisher = m_node_handle.advertise<geometry_msgs::Vector3>(TOPIC_VELOCITY_OUTPUT, 100, false);
}

void ImuAccumulator::scanCallback(const sensor_msgs::Imu::ConstPtr& imuData)
{
    if (!initSet)
    {
        initX = imuData->linear_acceleration.x;
        initY = imuData->linear_acceleration.y;
        initZ = imuData->linear_acceleration.z;
        initSet = true;
        lastValueTime = ros::Time::now();
        return;
    }

    geometry_msgs::Vector3 velocity;

    ros::Duration timeDiff = ros::Time::now() - lastValueTime;

    xAcc += (imuData->linear_acceleration.x - initX) * timeDiff.toSec();
    yAcc += (imuData->linear_acceleration.y - initY) * timeDiff.toSec();
    zAcc += (imuData->linear_acceleration.z - initZ) * timeDiff.toSec();

    velocity.x = xAcc;
    velocity.y = yAcc;
    velocity.z = zAcc;

    lastValueTime = ros::Time::now();

    m_speed_publisher.publish(velocity);
}
