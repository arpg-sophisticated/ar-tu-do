#include "wall_separation.h"


WallSeparation::WallSeparation()
    : m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallSeparation::lidar_callback, this);
    this->m_voxel_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(TOPIC_VOXEL_, 1, true);
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
    

    sensor_msgs::PointField tmp1 = sensor_msgs::PointField(); tmp1.name='x'; tmp1.offset=0;tmp1.datatype=sensor_msgs::PointField::FLOAT32;tmp1.count=2;
    sensor_msgs::PointField tmp2 = sensor_msgs::PointField(); tmp2.name='y'; tmp2.offset=4;tmp2.datatype=sensor_msgs::PointField::FLOAT32;tmp2.count=2;
    sensor_msgs::PointField tmp3 = sensor_msgs::PointField(); tmp3.name='score'; tmp3.offset=8;tmp3.datatype=sensor_msgs::PointField::UINT32;tmp3.count=1;

    
    voxelsCloud.fields = {tmp1,tmp2,tmp3};
    voxelsCloud.height = 1;
    voxelsCloud.point_step = 18; // FLOAT32 4Bytes * 2 (x und y Koordinate)+2 (count uint32)
    voxelsCloud.width = voxels.size();
    voxelsCloud.row_step = voxelsCloud.point_step * voxelsCloud.width;
    voxelsCloud.is_bigendian =true;
    voxelsCloud.is_dense = false;
    voxelsCloud.data.resize(voxelsCloud.point_step*voxelsCloud.width);


    for(size_t cp = 0; cp <voxelsCloud.width;++cp){
        uint32_t tmp = voxels[cp]->get_score();
        memcpy(&voxelsCloud.data[cp*voxelsCloud.point_step+voxelsCloud.fields[0].offset],&voxels[cp]->x,sizeof(double));
        memcpy(&voxelsCloud.data[cp*voxelsCloud.point_step+voxelsCloud.fields[1].offset],&voxels[cp]->y,sizeof(double));
        memcpy(&voxelsCloud.data[cp*voxelsCloud.point_step+voxelsCloud.fields[2].offset],&tmp,sizeof(uint32_t));
        std::cout<< "Voxel " <<cp <<": xKoordinate: " <<voxels[cp]->x << " yKoordinate: " << voxels[cp]->y << " Score: "<<tmp<<"\n";
    }


    m_voxel_publisher.publish(voxelsCloud);

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