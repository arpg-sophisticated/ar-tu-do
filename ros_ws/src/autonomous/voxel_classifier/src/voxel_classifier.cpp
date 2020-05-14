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
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(topicVoxels, 1, &VoxelClassifier::voxel_callback, this);

    this->m_cluster_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGBL>>(topicClusters, 1);

    m_epsilon = DEFAULT_EPSILON;
    m_minimum_points = DEFAULT_MINIMUM_POINTS;
    m_dyn_cfg_server.setCallback([&](voxel_classifier::voxel_classifierConfig& cfg, uint32_t) {
        m_epsilon = cfg.cluster_epsilon;
        m_minimum_points = cfg.cluster_minimum_points;
    });
}

void VoxelClassifier::voxel_callback(const sensor_msgs::PointCloud2::ConstPtr& voxelPointcloud)
{

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::fromROSMsg(*voxelPointcloud, *inputCloud);

    m_frame = voxelPointcloud->header.frame_id;

    std::vector<Point_> dbScanPoints;
    for (size_t i = 0; i < inputCloud->size(); i++)
    {
        Point_ tmp;
        tmp.x = (*inputCloud)[i].x;
        tmp.y = (*inputCloud)[i].y;
        tmp.z = (*inputCloud)[i].z;
        tmp.clusterID = -1;
        dbScanPoints.push_back(tmp);
    }

    DBSCAN ds(m_minimum_points, m_epsilon * m_epsilon, &dbScanPoints);
    ds.run();

    cluster_publish(&dbScanPoints);
}

void VoxelClassifier::cluster_publish(std::vector<Point_>* clusters)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr msg(new pcl::PointCloud<pcl::PointXYZRGBL>);
    msg->header.frame_id = m_frame;

    for (size_t i = 0; i < clusters->size(); i++)
    {
        pcl::PointXYZRGBL tmp;
        tmp.x = (*clusters)[i].x;
        tmp.y = (*clusters)[i].y;
        tmp.z = (*clusters)[i].z;
        tmp.label = (*clusters)[i].clusterID; // todo: we lose color information here!
        msg->push_back(tmp);
    }

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    m_cluster_publisher.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_classifier");
    VoxelClassifier voxelClassifier;
    ros::spin();
    return EXIT_SUCCESS;
}
