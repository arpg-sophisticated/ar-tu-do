#include "wall_separation.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <math.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

/** Why does this exist nowhere else :'( */
double euclideanLength(double vector[], size_t size)
{
    double norm = 0;
    for (size_t i = 0; i < size; i++)
        norm += vector[i] * vector[i];
    return sqrt(norm);
}

WallSeparation::WallSeparation()
    : m_private_node_handle("~")
    , m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    std::string topicLaserScan;
    std::string topicVoxel;

    if (!this->m_private_node_handle.getParamCached("topic_laser_scan", topicLaserScan))
        topicLaserScan = TOPIC_INPUT_POINTCLOUD;

    if (!this->m_private_node_handle.getParamCached("topic_voxels", topicVoxel))
        topicVoxel = TOPIC_VOXEL_;

    this->m_input_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicLaserScan, 1, &WallSeparation::input_callback, this);
    this->m_voxel_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicVoxel, 1, true);
}

void WallSeparation::input_callback(const sensor_msgs::PointCloud2::ConstPtr& pointCloud)
{

#if 0   
    std::vector<geometry_msgs::Point> points = lidar_to_cartesian(lidar);

    double voxelResolution;
    if (!this->m_private_node_handle.getParamCached("voxel_size", voxelResolution))
        voxelResolution = 0.2;

    double minimumPointDiscard = double(lidar->range_min / lidar->range_max);

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
        voxel->increment_score(
            point.z /
            double(
                lidar->range_max)); // The further the point is away, the higher it counts (because they scatter more)

        m_voxels[key] = voxel;
    }

    std::unordered_map<std::string, Voxel*>::iterator it = m_voxels.begin();
    std::vector<Voxel*> voxels;
    double scoreHolder[m_voxels.size()];
    int i = 0;
    while (it != m_voxels.end())
    {
        if (it->second->get_score() <= minimumPointDiscard)
        {
            delete it->second;
            it = m_voxels.erase(it);
        }
        else
        {
            voxels.push_back(it->second);
            scoreHolder[i++] = it->second->get_score();
            it++;
        }
    }

    double normedScores[voxels.size()];
    memcpy(normedScores, scoreHolder, voxels.size() * sizeof(double));
    double vectorEuclideanLength = euclideanLength(normedScores, voxels.size());

    // Anlegen der Parameter für jede Wert eines Feldes (Strukturangabe)
    sensor_msgs::PointField tmp1 = sensor_msgs::PointField();
    tmp1.name = 'x';
    tmp1.offset = 0;
    tmp1.datatype = sensor_msgs::PointField::FLOAT32;
    tmp1.count = 1;
    sensor_msgs::PointField tmp2 = sensor_msgs::PointField();
    tmp2.name = 'y';
    tmp2.offset = sizeof(float);
    tmp2.datatype = sensor_msgs::PointField::FLOAT32;
    tmp2.count = 1;
    sensor_msgs::PointField tmp3 = sensor_msgs::PointField();
    tmp3.name = "z";
    tmp3.offset = sizeof(float) + sizeof(float);
    tmp3.datatype = sensor_msgs::PointField::FLOAT32;
    tmp3.count = 1;
    sensor_msgs::PointField tmp4 = sensor_msgs::PointField();
    tmp4.name = "score";
    tmp4.offset = sizeof(float) + sizeof(float) + sizeof(float);
    tmp4.datatype = sensor_msgs::PointField::FLOAT32;
    tmp4.count = 1;

    // Message mit Metainformationen befüllen
    voxelsCloud.header.frame_id = LIDAR_FRAME;
    voxelsCloud.header.stamp = ros::Time::now();
    voxelsCloud.fields = { tmp1, tmp2, tmp3, tmp4 };
    voxelsCloud.height = 1;
    voxelsCloud.point_step = 4 * sizeof(float); // FLOAT32 4Bytes * 4 (x, y und z Koordinate und score float)
    voxelsCloud.width = voxels.size();
    voxelsCloud.row_step = voxelsCloud.point_step * voxelsCloud.width;
    voxelsCloud.is_bigendian = false;
    voxelsCloud.is_dense = false;
    voxelsCloud.data.resize(voxelsCloud.point_step * voxelsCloud.width);

    // Daten der Message zuführen TODO Effizienter machen Daten feld vorher
    // befüllen und nur zuweisen später?
    for (size_t cp = 0; cp < voxelsCloud.width; ++cp)
    {
        float tmp = float(voxels[cp]->get_score() / vectorEuclideanLength); // Norm it down here...
        memcpy(&voxelsCloud.data.data()[cp * voxelsCloud.point_step + voxelsCloud.fields[0].offset], &voxels[cp]->x,
               sizeof(float));
        memcpy(&voxelsCloud.data.data()[cp * voxelsCloud.point_step + voxelsCloud.fields[1].offset], &voxels[cp]->y,
               sizeof(float));
        voxelsCloud.data.data()[cp * voxelsCloud.point_step + voxelsCloud.fields[2].offset] = 0;

        memcpy(&voxelsCloud.data.data()[cp * voxelsCloud.point_step + voxelsCloud.fields[3].offset], &tmp,
               sizeof(float));
    }

    m_voxel_publisher.publish(voxelsCloud);

    this->m_debug_geometry.drawVoxels(0, voxels, voxelResolution, voxelResolution,
                                      1 / 10.0); // 10 points for maximum score
#endif
    double voxelResolution;
    if (!this->m_private_node_handle.getParamCached("voxel_size", voxelResolution))
        voxelResolution = 0.2;

    int meanK;
    if (!this->m_private_node_handle.getParamCached("sor_mean_k", meanK))
        meanK = 2;

    double stddevMulThresh;
    if (!this->m_private_node_handle.getParamCached("sor_stddev_mul_thresh", stddevMulThresh))
        voxelResolution = 3.0;

    bool removeOutliers;
    if (!this->m_private_node_handle.getParamCached("sor_enabled", removeOutliers))
        removeOutliers = false;

    // Container for original & filtered data
    pcl::PCLPointCloud2::Ptr inputCloud(new pcl::PCLPointCloud2);

    // Convert to PCL data type
    pcl_conversions::toPCL(*pointCloud, *(inputCloud));

    // Remove statistical outliers
    pcl::PCLPointCloud2::Ptr cloud_voxelized_ptr(new pcl::PCLPointCloud2);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud(inputCloud);
    vox.setLeafSize(voxelResolution, voxelResolution, voxelResolution);
    vox.filter(*cloud_voxelized_ptr);

    pcl::PCLPointCloud2::Ptr cloud_sor_ptr(new pcl::PCLPointCloud2);
    if (removeOutliers)
    {
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud_voxelized_ptr);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(stddevMulThresh);
        sor.filter(*cloud_sor_ptr);
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(removeOutliers ? *cloud_sor_ptr : *cloud_voxelized_ptr, output);

    // Publish the data
    this->m_voxel_publisher.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_separation");
    WallSeparation wallSeparation;
    ros::spin();
    return EXIT_SUCCESS;
}
