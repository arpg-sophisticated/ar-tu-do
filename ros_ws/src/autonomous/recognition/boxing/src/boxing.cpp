#include "boxing.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <math.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <unordered_map>

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
    /*this->m_colored_point_cloud_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicColoredPointCloud, 1, &Boxing::colored_input_callback,
                                                          this);
*/
    this->m_voxel_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(topicVoxel, 1, true);

    m_dyn_cfg_server.setCallback([&](boxing::boxingConfig& cfg, uint32_t) {
        m_voxel_size = cfg.voxel_size;
        m_lidar_percentage = cfg.lidar_point_percentage;
        m_filter_by_min_score_enabled = cfg.m_filter_by_min_score_enabled;
        m_filter_by_min_score = cfg.filter_by_min_score;
        m_sor_enabled = cfg.sor_enabled;
        m_sor_mean_k = cfg.sor_mean_k;
        m_sor_stddev_mul_thresh = cfg.sor_stddev_mul_thresh;

        m_colors_enabled = cfg.enable_colors;
        m_adjacent_voxels = cfg.adjacent_voxels;
        m_color_levels = cfg.color_levels;
        m_color_samples = cfg.color_samples;
    });
}

void Boxing::colored_input_callback(const sensor_msgs::PointCloud2::ConstPtr& coloredCloud)
{
    if (reference_frame.length() == 0)
        return;
    if (!m_colors_enabled)
        return;
    if (floatCompare(this->m_minimum_x, this->m_maximum_x) || floatCompare(this->m_minimum_y, this->m_maximum_y) ||
        floatCompare(this->m_minimum_z, this->m_maximum_z))
    {
        this->m_colored_cloud = NULL;
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colors(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*coloredCloud, *colors);
    this->m_colored_cloud = colors;
    this->m_colored_cloud_dirty = true;
}

void Boxing::preprocess_colored_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorsTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl_ros::transformPointCloud(reference_frame, *this->m_colored_cloud, *colorsTransformed, transform_listener);

    // crop this pointcloud as we only need the colours on the height of our voxels.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorsCropped(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setInputCloud(colorsTransformed);
    crop.setMin(
        Eigen::Vector4f(m_minimum_x - m_voxel_size, m_minimum_y - m_voxel_size, m_minimum_z - m_voxel_size, 1.0f));
    crop.setMax(
        Eigen::Vector4f(m_maximum_x + m_voxel_size, m_maximum_y + m_voxel_size, m_maximum_z + m_voxel_size, 1.0f));
    crop.filter(*colorsCropped);

    // now downsample using a voxel grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorsDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::RandomSample<pcl::PointXYZRGB> downsample;
    downsample.setInputCloud(colorsCropped);
    downsample.setSample(m_color_samples);
    downsample.filter(*colorsDownsampled);

    this->m_colored_cloud = colorsDownsampled;
    this->m_colored_cloud_dirty = false;
}

void Boxing::input_callback(const sensor_msgs::PointCloud2::ConstPtr& pointCloud)
{
    message_continuity_check.handleMessageSeq("Boxing: Laserscan: ", pointCloud->header.seq);

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

    std::unordered_map<uint64_t, pcl::PointXYZRGBL> quantized_cloud_voxel_map;

    this->m_maximum_x = 0;
    this->m_minimum_x = 0;
    this->m_maximum_y = 0;
    this->m_minimum_y = 0;
    this->m_maximum_z = 0;
    this->m_minimum_z = 0;

    size_t step_size = 1.0f / m_lidar_percentage;

    for (size_t i = 0; i < output_cloud->points.size(); i += step_size)
    {
        pcl::PointXYZ& point = output_cloud->at(i);
        float x = point.x - remainderf(point.x, m_voxel_size);
        float y = point.y - remainderf(point.y, m_voxel_size);
        float z = point.z; // - remainderf(point.z, m_voxel_size);

        if (point.x > this->m_maximum_x)
            this->m_maximum_x = point.x;
        if (point.x < this->m_minimum_x)
            this->m_minimum_x = point.x;

        if (point.y > this->m_maximum_y)
            this->m_maximum_y = point.y;
        if (point.y < this->m_minimum_y)
            this->m_minimum_y = point.y;

        if (point.z > this->m_maximum_z)
            this->m_maximum_z = point.z;
        if (point.z < this->m_minimum_z)
            this->m_minimum_z = point.z;

        // search the point we want to increment
        uint64_t voxel_id = get_voxel_id(x, y, z);
        auto foundPair = quantized_cloud_voxel_map.find(voxel_id);
        pcl::PointXYZRGBL* foundPoint = &(foundPair->second);
        bool found = foundPair != quantized_cloud_voxel_map.end();
        if (found)
        {
            foundPoint->label++;

            if (foundPoint->label > largest_intensity)
            {
                largest_intensity = foundPoint->label;
            }
        }
        else
        {
            pcl::PointXYZRGBL point;
            point.x = x;
            point.y = y;
            point.z = z;
            if (largest_intensity < 1)
            {
                largest_intensity = 1;
            }
            quantized_cloud_voxel_map[voxel_id] = point;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr quantized_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    quantized_cloud->header.frame_id = output_cloud->header.frame_id;
    quantized_cloud->points.resize(quantized_cloud_voxel_map.size());
    size_t i = 0;
    for (auto it : quantized_cloud_voxel_map)
    {
        pcl::PointXYZRGBL point = it.second;
        if (m_filter_by_min_score_enabled)
        {
            if ((point.label / largest_intensity) < m_filter_by_min_score)
                continue;
        }
        point.label = (point.label / largest_intensity) *
            static_cast<float>(std::numeric_limits<uint32_t>::max()); // uint32 max value

        quantized_cloud->points[i++] = point;
    }
    quantized_cloud->points.resize(i);

    if (m_colored_cloud && m_colors_enabled)
    {
        if (this->m_colored_cloud_dirty)
            this->preprocess_colored_cloud();

        // now we iterate over every point in the colored pointcloud and build a color histogram for
        // every voxel
        std::map<uint64_t, std::map<uint32_t, uint32_t>> histograms;

        std::map<uint32_t, uint32_t> global_histogram; // to determine the most used color for uncolored voxels

        for (size_t i = 0; i < m_colored_cloud->size(); i++)
        {
            pcl::PointXYZRGB& cp = m_colored_cloud->points[i];

            uint64_t voxel_id =
                get_voxel_id(cp.x - remainderf(cp.x, m_voxel_size), cp.y - remainderf(cp.y, m_voxel_size),
                             cp.z - remainderf(cp.z, m_voxel_size));

            // quantize colors!
            uint8_t r = round(((cp.r + 1) / 256.0) * m_color_levels) * (256 / m_color_levels);
            uint8_t g = round(((cp.g + 1) / 256.0) * m_color_levels) * (256 / m_color_levels);
            uint8_t b = round(((cp.b + 1) / 256.0) * m_color_levels) * (256 / m_color_levels);

            uint32_t rgb = (r << (16)) | g << (8) | (b);

            if (histograms.count(voxel_id) > 0 && histograms[voxel_id].count(rgb) > 0)
            {
                histograms[voxel_id][rgb] = histograms[voxel_id][rgb] + 1;
            }
            else
            {
                histograms[voxel_id][rgb] = 1;
            }

            if (global_histogram.count(rgb) > 0)
            {
                global_histogram[rgb] = global_histogram[rgb] + 1;
            }
            else
            {
                global_histogram[rgb] = 1;
            }
        }

        // determine global maximum color
        uint32_t global_maximum_color = 0, global_maximum_count = 0;
        for (auto& it : global_histogram)
        {
            if (it.second > global_maximum_count)
            {
                global_maximum_color = it.first;
                global_maximum_count = it.second;
            }
        }

        // with our histograms, we get the value which was found in most of the cases in our
        // histogram and choose it as the color of the voxel
        // optionally we also evaluate adjacent voxels.

        for (size_t j = 0; j < quantized_cloud->points.size(); j++)
        {
            float x = quantized_cloud->points[j].x;
            float y = quantized_cloud->points[j].y;
            float z = quantized_cloud->points[j].z;

            uint16_t largest_color_count = 0;
            uint32_t largest_rgb = 0;

            std::vector<uint128_t> voxel_ids;
            voxel_ids.push_back(get_voxel_id(x, y, z));

            if (m_adjacent_voxels)
            {
                for (int8_t xA = -1; xA <= 1; xA++)
                {
                    for (int8_t yA = -1; yA <= 1; yA++)
                    {
                        for (int8_t zA = -1; zA <= 1; zA++)
                        {
                            voxel_ids.push_back(
                                get_voxel_id(x + xA * m_voxel_size, y + yA * m_voxel_size, z + zA * m_voxel_size));
                        }
                    }
                }
            }

            for (auto& vid : voxel_ids)
            {
                if (histograms.count(vid) == 0 || histograms[vid].size() == 0)
                    continue;
                for (auto& it : histograms[vid])
                {
                    if (it.second > largest_color_count)
                    {
                        largest_rgb = it.first;
                        largest_color_count = it.second;
                    }
                }
            }

            if (largest_color_count == 0)
            {
                // no matching colors in that voxel, pull global maximum for this...
                largest_rgb = global_maximum_color;
            }

            quantized_cloud->points[j].r = (largest_rgb >> 16) & 0xff;
            quantized_cloud->points[j].g = (largest_rgb >> 8) & 0xff;
            quantized_cloud->points[j].b = (largest_rgb >> 0) & 0xff;
            quantized_cloud->points[j].a = 255;
        }
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
