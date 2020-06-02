#include "wall_detection.h"
#include <dynamic_reconfigure/server.h>
#include <wall_detection/wall_detectionConfig.h>

WallDetection::WallDetection()
    : m_private_node_handle("~")
{
    std::string topicClusters;
    std::string topicWalls;
    std::string topicObstacles;

    if (!this->m_private_node_handle.getParamCached("topic_input_clusters", topicClusters))
        topicClusters = TOPIC_VOXEL_;

    if (!this->m_private_node_handle.getParamCached("topic_output_walls", topicWalls))
        topicWalls = TOPIC_WALLS_;

    if (!this->m_private_node_handle.getParamCached("topic_output_obstacles", topicObstacles))
        topicObstacles = topicObstacles;

    this->m_voxel_subscriber =
        m_node_handle.subscribe<pcl::PointCloud<pcl::PointXYZRGBL>>(topicClusters, 1,
                                                                    &WallDetection::wallDetection_callback, this);

    this->m_wall_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGBL>>(topicWalls, 1);

    this->m_obstacles_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGBL>>(topicObstacles, 1);

    m_wall_radius = 3;

    m_dyn_cfg_server.setCallback(
        [&](wall_detection::wall_detectionConfig& cfg, uint32_t) { m_wall_radius = cfg.wall_search_radius; });
}

void WallDetection::wallDetection_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& inputVoxels)
{

    frameID = inputVoxels->header.frame_id;

    std::unordered_map<int, std::vector<pcl::PointXYZRGBL>*> clustersUsed;
    // map cluster ids in label to map with cluster id as key and pointvector as value
    for (size_t i = 0; i < inputVoxels->points.size(); i++)
    {
        if (clustersUsed.count(inputVoxels->points[i].label) > 0)
        {
            clustersUsed[inputVoxels->points[i].label]->push_back(inputVoxels->points[i]);
        }
        else
        {
            std::vector<pcl::PointXYZRGBL>* tmp = new std::vector<pcl::PointXYZRGBL>();
            tmp->push_back(inputVoxels->points[i]);
            clustersUsed.insert({ inputVoxels->points[i].label, tmp });
        }
    }

    // determine maximum left and right clusters in a radius
    auto test = determineWallIDs(clustersUsed, m_wall_radius); // these ids are the walls

    // publish only the clusters with ids equal to the walls, but only if the id is > 0
    if (test.first >= 0 && test.second >= 0)
        publishWall(clustersUsed[test.first], clustersUsed[test.second]);

    // publish all other clusters
    publishObstacles(clustersUsed, test);

    // clean up
    for (auto itr = clustersUsed.begin(); itr != clustersUsed.end(); ++itr)
        delete itr->second;
}

void WallDetection::publishObstacles(std::unordered_map<int, std::vector<pcl::PointXYZRGBL>*> mapClusters,
                                     std::pair<int, int> wallIDs)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr msg(new pcl::PointCloud<pcl::PointXYZRGBL>);

    msg->header.frame_id = frameID;

    for (auto itr = mapClusters.begin(); itr != mapClusters.end(); ++itr)
    {
        if (itr->first != wallIDs.first && itr->first != wallIDs.second)
        {
            for (size_t i = 0; i < itr->second->size(); i++)
            {
                msg->push_back((*itr->second)[i]);
            }
        }
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    m_obstacles_publisher.publish(msg);
}

void WallDetection::publishWall(std::vector<pcl::PointXYZRGBL>* wallLeft, std::vector<pcl::PointXYZRGBL>* wallRight)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr msg(new pcl::PointCloud<pcl::PointXYZRGBL>);
    msg->header.frame_id = frameID;

    for (size_t i = 0; i < wallLeft->size(); i++)
    {
        pcl::PointXYZRGBL tmp;
        tmp.x = (*wallLeft)[i].x;
        tmp.y = (*wallLeft)[i].y;
        tmp.z = (*wallLeft)[i].z;
        tmp.r = (*wallLeft)[i].r;
        tmp.g = (*wallLeft)[i].g;
        tmp.b = (*wallLeft)[i].b;

        tmp.label = 1;

        msg->push_back(tmp);
    }

    for (size_t i = 0; i < wallRight->size(); i++)
    {
        pcl::PointXYZRGBL tmp;
        tmp.x = (*wallRight)[i].x;
        tmp.y = (*wallRight)[i].y;
        tmp.z = (*wallRight)[i].z;
        tmp.r = (*wallRight)[i].r;
        tmp.g = (*wallRight)[i].g;
        tmp.b = (*wallRight)[i].b;
        tmp.label = 0;

        msg->push_back(tmp);
    }

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    m_wall_publisher.publish(msg);
}

std::pair<int, int> WallDetection::determineWallIDs(std::unordered_map<int, std::vector<pcl::PointXYZRGBL>*> mapToCheck,
                                                    float radius)
{
    float maxLeft = 0;
    float maxRight = 0;
    int maxLeftID = -1;
    int maxRightID = -1;

    for (auto itr = mapToCheck.begin(); itr != mapToCheck.end(); ++itr)
    {
        for (auto itrVector = itr->second->begin(); itrVector != itr->second->end(); ++itrVector)
        {
            if ((itrVector->y > maxLeft) && (fabsf(itrVector->x) <= radius))
            {
                maxLeft = itrVector->x;
                maxLeftID = itrVector->label;
            }
            if ((itrVector->y < maxRight) && fabsf((itrVector->x) <= radius))
            {
                maxRight = itrVector->x;
                maxRightID = itrVector->label;
            }
        }
    }
    return std::pair<int, int>(maxLeftID, maxRightID);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_detection");
    WallDetection wallDetection;
    ros::spin();
    return EXIT_SUCCESS;
}
