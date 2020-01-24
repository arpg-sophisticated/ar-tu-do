#include "wall_separation.h"

WallSeparation::WallSeparation()
    : m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallSeparation::lidar_callback, this);
    this->m_voxel_publisher = m_node_handle.advertise<geometry_msgs::Vector3>(TOPIC_VOXEL_, 1, true);
}

std::vector<geometry_msgs::Point> lidar_to_cartesian(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    std::vector<geometry_msgs::Point> points;

    uint32_t scan_count = (lidar->angle_max - lidar->angle_min) / lidar->angle_increment;

    for (uint32_t i = 0; i < scan_count; i++)
    {
        double range = lidar->ranges[i];
        double angle = lidar->angle_min + i * lidar->angle_increment;

        geometry_msgs::Point point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        points.push_back(point);
    }

    return points;
}

void WallSeparation::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    std::vector<geometry_msgs::Point> points = lidar_to_cartesian(lidar);

    float voxelResolution = 0.2f;

    for (auto pair : m_voxels)
        pair.second->start_new_episode();

    for (geometry_msgs::Point point : points)
    {
        double voxelX = point.x - remainder(point.x, voxelResolution);
        double voxelY = point.y - remainder(point.y, voxelResolution);
        char buf[100];
        sprintf(buf, "%f-%f", voxelX, voxelY);
        std::string key = buf;
        Voxel* voxel;
        if (m_voxels.find(key) != m_voxels.end())
        {
            voxel = m_voxels[key];
        }
        else
        {
            voxel = new Voxel();
            voxel->start_new_episode();
            voxel->x = voxelX;
            voxel->y = voxelY;
        }
        voxel->increment_score();

        m_voxels[key] = voxel;
    }

    std::unordered_map<std::string, Voxel*>::iterator it = m_voxels.begin();
    std::vector<Voxel*> voxels;
    while (it != m_voxels.end())
    {
        if (it->second->get_score() == 0)
        {
            delete it->second;
            it = m_voxels.erase(it);
        }
        else
        {
            voxels.push_back(it->second);
            it++;
        }
    }

    this->m_debug_geometry.drawVoxels(0, voxels, voxelResolution, voxelResolution,
                                      1 / 10.0f); // 10 points for maximum score
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_separation");
    WallSeparation wallSeparation;
    ros::spin();
    return EXIT_SUCCESS;
}