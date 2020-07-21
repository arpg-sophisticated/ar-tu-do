#include "voxel_classifier.h"
#include <dynamic_reconfigure/server.h>
#include <voxel_classifier/voxel_classifierConfig.h>

#define DEFAULT_MINIMUM_POINTS (4)
#define DEFAULT_EPSILON (0.47)

VoxelClassifier::VoxelClassifier()
    : m_private_node_handle("~")
{
    std::string topicVoxels;
    std::string topicClusters;

    if (!this->m_private_node_handle.getParamCached("topic_input_voxels", topicVoxels))
        topicVoxels = TOPIC_VOXEL_;

    if (!this->m_private_node_handle.getParamCached("topic_output_clusters", topicClusters))
        topicClusters = TOPIC_CLUSTER_;

    this->m_voxel_subscriber =
        m_node_handle.subscribe<pcl::PointCloud<pcl::PointXYZRGBL>>(topicVoxels, 1, &VoxelClassifier::voxel_callback,
                                                                    this);

    this->m_cluster_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGBL>>(topicClusters, 1);

    m_epsilon = DEFAULT_EPSILON;
    m_minimum_points = DEFAULT_MINIMUM_POINTS;
    m_dyn_cfg_server.setCallback([&](voxel_classifier::voxel_classifierConfig& cfg, uint32_t) {
        m_epsilon = cfg.cluster_epsilon;
        m_minimum_points = cfg.cluster_minimum_points;
        m_color_weight = cfg.color_weight;
    });
}

void VoxelClassifier::voxel_callback(pcl::PointCloud<pcl::PointXYZRGBL> inputCloud)
{
    for (size_t i = 0; i < inputCloud.size(); i++)
        inputCloud.at(i).label = UNCLASSIFIED;

    DBSCAN ds(m_minimum_points, m_epsilon * m_epsilon, m_color_weight, &inputCloud);
    ds.run();

    for (auto pointIterator = inputCloud.begin(); pointIterator != inputCloud.end();)
    {
        if (pointIterator->label == NOISE)
        {
            pointIterator = inputCloud.erase(pointIterator);
        }
        else
        {
            ++pointIterator;
        }
    }

    m_cluster_publisher.publish(inputCloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_classifier");
    VoxelClassifier voxelClassifier;
    ros::spin();
    return EXIT_SUCCESS;
}
