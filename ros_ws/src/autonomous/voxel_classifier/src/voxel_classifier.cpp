#include "voxel_classifier.h"

#define MINIMUM_POINTS 4
#define EPSILON (0.47 * 0.47)

VoxelClassifier::VoxelClassifier()
    : m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION_Cluster, LIDAR_FRAME)
{
    this->m_voxel_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(TOPIC_VOXEL_, 1, &VoxelClassifier::voxel_callback, this);

    this->m_cluster_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZI>>(TOPIC_CLUSTER_, 1);
}
/*std::vector<Point> VoxelClassifier::transformPoints(){
  vector<Point> r_points;
  for (size_t i = 0; i < voxels.size(); i++)
  {
     Point tmp;
     tmp.x=voxels[i].x;
     tmp.y=voxels[i].y;
     tmp.z=0;
     tmp.clusterID=0;
     tmp.clusterID=UNCLASSIFIED;
     r_points.push_back(tmp);
  }
  return r_points;
}*/

void VoxelClassifier::voxel_callback(const sensor_msgs::PointCloud2::ConstPtr& voxelPointcloud)
{
    voxels.clear();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*voxelPointcloud, *inputCloud);

    m_frame = voxelPointcloud->header.frame_id;

    for (size_t i = 0; i < inputCloud->size(); i++)
    {
        Voxel tmp;
        tmp.x = (*inputCloud)[i].x;
        tmp.y = (*inputCloud)[i].y;
        tmp.z = (*inputCloud)[i].z;
        tmp.setScore((*inputCloud)[i].intensity);
        tmp.clusterID = -1;
        voxels.push_back(tmp);
    }

    DBSCAN ds(MINIMUM_POINTS, EPSILON, &voxels);
    ds.run();
    // printResults(voxels, voxels.size());
    cluster_publish();
}

void VoxelClassifier::cluster_publish()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);
    msg->header.frame_id = m_frame;
    msg->height = 1;
    msg->width = voxels.size();
    // msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

    for (size_t i = 0; i < voxels.size(); i++)
    {
        pcl::PointXYZI tmp;
        tmp.x = voxels[i].x;
        tmp.y = voxels[i].y;
        tmp.z = voxels[i].z;
        tmp.intensity = voxels[i].clusterID;
        msg->push_back(tmp);
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    m_cluster_publisher.publish(msg);
}

void VoxelClassifier::printResults(vector<Voxel>& points, int num_points)
{
    int i = 0;
    printf("Number of points: %u\n"
           " x     y     z     cluster_id\n"
           "-----------------------------\n",
           num_points);
    while (i < num_points)
    {
        printf("%5.2lf %5.2lf %5.2lf: %d\n", points[i].x, points[i].y, points[i].z, points[i].clusterID);
        ++i;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_classifier");
    VoxelClassifier voxelClassifier;
    ros::spin();
    return EXIT_SUCCESS;
}
