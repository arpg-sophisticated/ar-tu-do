#include "boxing.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <math.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

/** Why does this exist nowhere else :'( */
/*double euclideanLength(double vector[], size_t size)
{
    double norm = 0;
    for (size_t i = 0; i < size; i++)
        norm += vector[i] * vector[i];
    return sqrt(norm);
}*/

Boxing::Boxing()
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
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicLaserScan, 1, &Boxing::input_callback, this);
    this->m_voxel_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicVoxel, 1, true);
}

void Boxing::input_callback(const sensor_msgs::PointCloud2::ConstPtr& pointCloud)
{
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL data type
    pcl::fromROSMsg(*pointCloud, *(inputCloud));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Perform the actual filtering
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(inputCloud);
    vox.setLeafSize(voxelResolution * 0.5, voxelResolution * 0.5, voxelResolution * 0.5);
    vox.filter(*cloud_voxelized_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (removeOutliers)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_voxelized_ptr);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(stddevMulThresh);
        sor.filter(*cloud_sor_ptr);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud = removeOutliers ? cloud_sor_ptr : cloud_voxelized_ptr;

    // now quantize this cloud for our system. this could probably also be implemented as a proper filter

    pcl::PointCloud<pcl::PointXYZ>::Ptr quantized_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *quantized_cloud += *output_cloud;
    quantized_cloud->header.frame_id = output_cloud->header.frame_id;
    for (size_t i = 0; i < quantized_cloud->points.size(); i++)
    {
        quantized_cloud->points[i].x =
            quantized_cloud->at(i).x - remainderf(quantized_cloud->points[i].x, voxelResolution);
        quantized_cloud->points[i].y =
            quantized_cloud->at(i).y - remainderf(quantized_cloud->points[i].y, voxelResolution);
        quantized_cloud->points[i].z =
            quantized_cloud->at(i).z - remainderf(quantized_cloud->points[i].z, voxelResolution);
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*quantized_cloud, output);

    // Publish the data
    this->m_voxel_publisher.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "boxing");
    Boxing wallSeparation;
    ros::spin();
    return EXIT_SUCCESS;
}
