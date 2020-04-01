#include "boxing.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
// mysterious code stolen from https://stackoverflow.com/a/15012792
bool floatCompare(double x, double y)
{
    double maxXYOne = std::max({ 1.0, std::fabs(x), std::fabs(y) });

    return std::fabs(x - y) <= std::numeric_limits<double>::epsilon() * maxXYOne;
}

Boxing::Boxing()
    : m_private_node_handle("~")
    , m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    std::string topicLaserScan;
    std::string topicVoxel;

    if (!this->m_private_node_handle.getParamCached("topic_input_cloud", topicLaserScan))
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
        stddevMulThresh = 3.0;

    bool removeOutliers;
    if (!this->m_private_node_handle.getParamCached("sor_enabled", removeOutliers))
        removeOutliers = false;

    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL data type
    pcl::fromROSMsg(*pointCloud, *(inputCloud));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (removeOutliers)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(inputCloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(stddevMulThresh);
        sor.filter(*cloud_sor_ptr);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud = removeOutliers ? cloud_sor_ptr : inputCloud;

    // now quantize this cloud for our system. this could probably also be implemented as a proper filter

    float largest_intensity = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr quantized_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    quantized_cloud->clear();
    quantized_cloud->header.frame_id = output_cloud->header.frame_id;
    for (size_t i = 0; i < output_cloud->points.size(); i++)
    {
        pcl::PointXYZ& point = output_cloud->at(i);
        float x = point.x - remainderf(point.x, voxelResolution);
        float y = point.y - remainderf(point.y, voxelResolution);
        float z = point.z - remainderf(point.z, voxelResolution);

        // search the point we want to increment
        bool found = false;
        for (size_t j = 0; j < quantized_cloud->points.size(); j++)
        {
            pcl::PointXYZI& foundPoint = quantized_cloud->points[j];
            if (floatCompare((double)foundPoint.x, (double)x) && floatCompare((double)foundPoint.y, (double)y) &&
                floatCompare((double)foundPoint.z, (double)z))
            {
                found = true;
                quantized_cloud->points[j].intensity++;

                if (quantized_cloud->points[j].intensity > largest_intensity)
                {
                    largest_intensity = quantized_cloud->points[j].intensity;
                }
                break;
            }
        }

        if (!found)
        {
            quantized_cloud->points.resize(quantized_cloud->points.size() + 1);
            quantized_cloud->points[quantized_cloud->points.size() - 1].x = x;
            quantized_cloud->points[quantized_cloud->points.size() - 1].y = y;
            quantized_cloud->points[quantized_cloud->points.size() - 1].z = z;
            quantized_cloud->points[quantized_cloud->points.size() - 1].intensity = 1;
            if (largest_intensity < 1)
            {
                largest_intensity = 1;
            }
        }
    }

    for (size_t i = 0; i < quantized_cloud->points.size(); i++)
    {
        quantized_cloud->points[i].intensity = quantized_cloud->points[i].intensity / largest_intensity;
    }

    double minimum_score;
    if (!this->m_private_node_handle.getParamCached("filter_by_score_minimum_score", minimum_score))
        minimum_score = 0.5;

    bool filter_by_score;
    if (!this->m_private_node_handle.getParamCached("filter_by_score_enabled", filter_by_score))
        filter_by_score = false;

    if (filter_by_score)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        for (size_t i = 0; i < quantized_cloud->points.size(); i++)
        {
            if (quantized_cloud->points[i].intensity < (float)minimum_score) // e.g. remove all pts below zAvg
            {
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(quantized_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*quantized_cloud);
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
