#include "boxing.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>

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
    std::string topicColoredPointCloud;

    if (!this->m_private_node_handle.getParamCached("topic_input_cloud", topicLaserScan))
        topicLaserScan = TOPIC_INPUT_POINTCLOUD;

    if (!this->m_private_node_handle.getParamCached("topic_voxels", topicVoxel))
        topicVoxel = TOPIC_VOXEL_;

    if (!this->m_private_node_handle.getParamCached("topic_input_colored_cloud", topicColoredPointCloud))
        topicColoredPointCloud = TOPIC_INPUT_COLORED_CLOUD;

    this->m_input_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicLaserScan, 1, &Boxing::input_callback, this);
    this->m_colored_point_cloud_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicColoredPointCloud, 1, &Boxing::colored_input_callback,
                                                          this);

    this->m_voxel_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicVoxel, 1, true);

    m_dyn_cfg_server.setCallback([&](boxing::boxingConfig& cfg, uint32_t) {
        m_voxel_size = cfg.voxel_size;
        m_filter_by_min_score_enabled = cfg.m_filter_by_min_score_enabled;
        m_filter_by_min_score = cfg.filter_by_min_score;
        m_sor_enabled = cfg.sor_enabled;
        m_sor_mean_k = cfg.sor_mean_k;
        m_sor_stddev_mul_thresh = cfg.sor_stddev_mul_thresh;
    });
}

void Boxing::colored_input_callback(const sensor_msgs::PointCloud2::ConstPtr& coloredCloud)
{
    if (reference_frame.length() == 0)
        return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colors(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorsTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*coloredCloud, *colors);

    pcl_ros::transformPointCloud(reference_frame, *colors, *colorsTransformed, transform_listener);

    this->m_colored_cloud = colorsTransformed;
}

void Boxing::input_callback(const sensor_msgs::PointCloud2::ConstPtr& pointCloud)
{

    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL data type
    pcl::fromROSMsg(*pointCloud, *(inputCloud));
    this->reference_frame = inputCloud->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (m_sor_enabled)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(inputCloud);
        sor.setMeanK(m_sor_mean_k);
        sor.setStddevMulThresh(m_sor_stddev_mul_thresh);
        sor.filter(*cloud_sor_ptr);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud = m_sor_enabled ? cloud_sor_ptr : inputCloud;

    // now quantize this cloud for our system. this could probably also be implemented as a proper filter

    float largest_intensity = 0;

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr quantized_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    quantized_cloud->clear();
    quantized_cloud->header.frame_id = output_cloud->header.frame_id;
    for (size_t i = 0; i < output_cloud->points.size(); i++)
    {
        pcl::PointXYZ& point = output_cloud->at(i);
        float x = point.x - remainderf(point.x, m_voxel_size);
        float y = point.y - remainderf(point.y, m_voxel_size);
        float z = point.z - remainderf(point.z, m_voxel_size);

        // search the point we want to increment
        bool found = false;
        for (size_t j = 0; j < quantized_cloud->points.size(); j++)
        {
            pcl::PointXYZRGBL& foundPoint = quantized_cloud->points[j];
            if (floatCompare((double)foundPoint.x, (double)x) && floatCompare((double)foundPoint.y, (double)y) &&
                floatCompare((double)foundPoint.z, (double)z))
            {
                found = true;
                quantized_cloud->points[j].label++;

                if (quantized_cloud->points[j].label > largest_intensity)
                {
                    largest_intensity = quantized_cloud->points[j].label;
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
            quantized_cloud->points[quantized_cloud->points.size() - 1].a = 255;
            if (largest_intensity < 1)
            {
                largest_intensity = 1;
            }

            if (m_colored_cloud)
            {
                // dirty O(n²) code ahead: we iterate over every point in the point cloud and if it is in the bounding
                // box, it is added to a unordered map or it's value is incremented by 1. In the end we choose the key
                // with the highest value. Thus, we get the mode (?)
                std::unordered_map<uint32_t, uint16_t> colors_in_voxel;

                pcl::CropBox<pcl::PointXYZRGB> crop;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropOut(new pcl::PointCloud<pcl::PointXYZRGB>);
                crop.setMin(Eigen::Vector4f(x - m_voxel_size, y - m_voxel_size, z - m_voxel_size, 0.0));
                crop.setMax(Eigen::Vector4f(x + m_voxel_size, y + m_voxel_size, z + m_voxel_size, 0.0));
                crop.setInputCloud(m_colored_cloud);
                crop.filter(*cropOut);

                for (size_t j = 0; j < cropOut->size(); j++)
                {
                    pcl::PointXYZRGB& cp = cropOut->points[j];

                    uint8_t quantization_level = 5; // 255 / 2^quantization_level
                    // 1 = 128 per color, 2 = 64 per color, 3 = 32 per color, 4 = 16 per color, 5 = 8 per color

                    uint8_t r = cp.r >> quantization_level;
                    uint8_t g = cp.g >> quantization_level;
                    uint8_t b = cp.b >> quantization_level;

                    uint32_t rgb =
                        (r << (16 + quantization_level)) | g << (8 + quantization_level) | (b << quantization_level);

                    if (colors_in_voxel.count(rgb) > 0)
                    {
                        colors_in_voxel[rgb] = colors_in_voxel[rgb] + 1;
                    }
                    else
                    {
                        colors_in_voxel[rgb] = 1;
                    }
                }

                uint16_t largest_color_count;
                uint32_t largest_rgb = 0;

                for (auto& it : colors_in_voxel)
                {
                    if (it.second > largest_color_count)
                    {
                        largest_rgb = it.first;
                        largest_color_count = it.second;
                    }
                }

                if (largest_rgb == 0)
                    largest_rgb = 255 << 16 | 255 << 8 | 255;

                colors_in_voxel.clear();

                quantized_cloud->points[quantized_cloud->points.size() - 1].b = (largest_rgb >> 16) & 0xff;
                quantized_cloud->points[quantized_cloud->points.size() - 1].g = (largest_rgb >> 8) & 0xff;
                quantized_cloud->points[quantized_cloud->points.size() - 1].r = (largest_rgb >> 0) & 0xff;

                quantized_cloud->points[quantized_cloud->points.size() - 1].a = 255;
            }
        }
    }

    for (size_t i = 0; i < quantized_cloud->points.size(); i++)
    {
        quantized_cloud->points[i].label =
            (quantized_cloud->points[i].label / largest_intensity) * 4294967295.0; // uint32 max value
    }

    if (m_filter_by_min_score_enabled)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
        for (size_t i = 0; i < quantized_cloud->points.size(); i++)
        {
            if (quantized_cloud->points[i].label <
                m_filter_by_min_score * 4294967295.0) // e.g. remove all pts below zAvg
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
