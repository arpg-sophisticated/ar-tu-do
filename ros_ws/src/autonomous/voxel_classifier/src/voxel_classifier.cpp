#include "voxel_classifier.h"

VoxelClassifier::VoxelClassifier()
{
    this->m_voxel_subscriber =
        m_node_handle.subscribe<sensor_msgs::PointCloud2>(TOPIC_VOXEL_, 1, &VoxelClassifier::voxel_callback, this);
}

void VoxelClassifier::voxel_callback(const sensor_msgs::PointCloud2::ConstPtr& voxelPointcloud)
{
    std::vector<Voxel> voxels;

    for (size_t i = 0; i < voxelPointcloud->width; i++)
    {
        Voxel tmp;
        uint32_t tmp_score;
        memcpy(&tmp.x, &voxelPointcloud->data[i * voxelPointcloud->point_step + voxelPointcloud->fields[0].offset],
               sizeof(float));
        memcpy(&tmp.y, &voxelPointcloud->data[i * voxelPointcloud->point_step + voxelPointcloud->fields[1].offset],
               sizeof(float));
        memcpy(&tmp_score, &voxelPointcloud->data[i * voxelPointcloud->point_step + voxelPointcloud->fields[2].offset],
               sizeof(uint32_t));
        tmp.setScore(tmp_score);
        voxels.push_back(tmp);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_classifier");
    VoxelClassifier voxelClassifier;
    ros::spin();
    return EXIT_SUCCESS;
}