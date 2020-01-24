#include "wall_separation.h"

WallSeparation::WallSeparation() : m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME) {
    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallSeparation::lidar_callback, this);
    

}

void WallSeparation::lidar_callback(const sensor_msgs::LaserScan::ConstPtr &lidar) {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_separation");
    WallSeparation wallSeparation;
    ros::spin();
    return EXIT_SUCCESS;

}