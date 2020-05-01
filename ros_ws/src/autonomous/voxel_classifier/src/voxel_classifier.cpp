#include "voxel_classifier.h"

#define MINIMUM_POINTS 4
#define EPSILON (0.47 * 0.47)

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

    this->m_cluster_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZI>>(topicClusters, 1);
}

void VoxelClassifier::voxel_callback(const sensor_msgs::PointCloud2::ConstPtr& voxelPointcloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
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

    DBSCAN ds(MINIMUM_POINTS, EPSILON, &dbScanPoints);
    ds.run();

    cluster_publish(&dbScanPoints);
}

void VoxelClassifier::cluster_publish(std::vector<Point_>* clusters)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);
    msg->header.frame_id = m_frame;

    for (size_t i = 0; i < clusters->size(); i++)
    {
        pcl::PointXYZI tmp;
        tmp.x = (*clusters)[i].x;
        tmp.y = (*clusters)[i].y;
        tmp.z = (*clusters)[i].z;
        tmp.intensity = (*clusters)[i].clusterID;
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
