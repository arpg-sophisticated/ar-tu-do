#include "wallfollowing.h"

Wallfollowing::Wallfollowing()
{
    m_laserscan_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &Wallfollowing::laserScanCallback, this);
}

void Wallfollowing::follow_walls()
{
}

void Wallfollowing::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
}

int main(int argc, char** argv)
{
    return 0;
}