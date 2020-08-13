#include "avoid_static_obstacles.h"
#include <dynamic_reconfigure/server.h>
#include <voxel_classifier/voxel_classifierConfig.h>

StaticObstacles::StaticObstacles()
    : m_private_node_handle("~")
{
    this->m_wall_subscriber =
        m_node_handle.subscribe<pcl::PointCloud<pcl::PointXYZRGBL>>(TOPIC_WALLS_, 1, &StaticObstacles::wall_callback,
                                                                    this);

    this->m_obstacles_subscriber =
        m_node_handle.subscribe<pcl::PointCloud<pcl::PointXYZRGBL>>(TOPIC_OBSTACLES_, 1,
                                                                    &StaticObstacles::obstacles_callback, this);
    this->m_newWall_publisher = m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGBL>>(TOPIC_NEWWALL_, 1);
}

void StaticObstacles::wall_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPointcloud)
{
    m_frame = wallPointcloud->header.frame_id;

    int closestObstacleID = getFirstObstacle();
    if (closestObstacleID != -1)
    {
        std::pair<int, pcl::PointXYZRGBL> avoidInfo = getObstacleVoxel(closestObstacleID, wallPointcloud);

        createLine(getClosestWallVoxel(avoidInfo.first, wallPointcloud), avoidInfo.second, wallPointcloud);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr obstacles_wall(new pcl::PointCloud<pcl::PointXYZRGBL>);

        obstacles_wall->header.frame_id = m_frame;
        for (size_t i = 0; i < wallPointcloud->points.size(); i++)
        {
            obstacles_wall->points.push_back(wallPointcloud->points[i]);
        }
        pcl_conversions::toPCL(ros::Time::now(), obstacles_wall->header.stamp);
        m_newWall_publisher.publish(obstacles_wall);
    }
}

void StaticObstacles::obstacles_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& obstaclesPointcloud)
{

    clustersUsed.clear();
    // map cluster ids in intensity to map with cluster id as key and pointvector as value
    for (size_t i = 0; i < obstaclesPointcloud->points.size(); i++)
    {
        if (clustersUsed.count(obstaclesPointcloud->points[i].label) > 0)
        {
            clustersUsed[obstaclesPointcloud->points[i].label]->push_back(obstaclesPointcloud->points[i]);
        }
        else
        {
            std::vector<pcl::PointXYZRGBL>* tmp = new std::vector<pcl::PointXYZRGBL>();
            tmp->push_back(obstaclesPointcloud->points[i]);
            clustersUsed.insert({ obstaclesPointcloud->points[i].label, tmp });
        }
    }
}

int StaticObstacles::getFirstObstacle()
{
    int closestObstacleID = -1;
    float closestObstacleDistance = MAXFLOAT;

    for (auto& iter : clustersUsed)
    {
        for (auto innerIter : *(iter.second))
        {
            if (innerIter.x < closestObstacleDistance && innerIter.x > 0)
            {
                closestObstacleDistance = innerIter.x;
                closestObstacleID = innerIter.label;
            }
        }
    }

    return closestObstacleID;
}

std::pair<int, pcl::PointXYZRGBL> StaticObstacles::getObstacleVoxel(
    int obstacleID, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints)
{
    std::pair<float, pcl::PointXYZRGBL> zeroID = getClosestDistanceToWall(0, obstacleID, wallPoints);
    std::pair<float, pcl::PointXYZRGBL> oneID = getClosestDistanceToWall(1, obstacleID, wallPoints);

    if (oneID.first > zeroID.first)
    {
        pcl::PointXYZRGBL returnVoxel = oneID.second;
        return { 1, returnVoxel };
    }
    else
    {
        pcl::PointXYZRGBL returnVoxel = zeroID.second;
        return { 0, returnVoxel };
    }
}

std::pair<float, pcl::PointXYZRGBL> StaticObstacles::getClosestDistanceToWall(
    int wallSide, int obstacleID, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints)
{
    float minDistance = MAXFLOAT;
    pcl::PointXYZRGBL closestPoint;

    if (obstacleID != -1)
    {
        for (auto& obstaclesVoxels : *(clustersUsed[obstacleID]))
        {
            for (auto& iter : wallPoints->points)
            {
                if (iter.label == wallSide)
                {
                    if (minDistance > getDistance(obstaclesVoxels, iter))
                    {
                        closestPoint = obstaclesVoxels;
                        minDistance = getDistance(obstaclesVoxels, iter);
                    }
                }
            }
        }
        return { minDistance, closestPoint };
    }
    else
    {
        return { -1, closestPoint };
    }
}

pcl::PointXYZRGBL StaticObstacles::getClosestWallVoxel(int WallID,
                                                       const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints)
{ // TODO FEHLERFÄLLE
    float closestDistanceValue = MAXFLOAT;
    pcl::PointXYZRGBL closestDistanceVoxel;

    for (auto& iter : wallPoints->points)
    {
        pcl::PointXYZRGBL origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 1.0;

        if (iter.label == (1 - WallID) && std::abs(iter.x) <= CLOSESTVOXELINTERVALL)
        {
            if (getDistance(origin, iter) < closestDistanceValue)
            {
                closestDistanceValue = getDistance(origin, iter);
                closestDistanceVoxel = iter;
            }
        }
    }

    return closestDistanceVoxel;
}

float StaticObstacles::getDistance(pcl::PointXYZRGBL obstacle, pcl::PointXYZRGBL wall)
{
    return std::sqrt(pow(obstacle.x - wall.x, 2) + pow(obstacle.y - wall.y, 2));
}

void StaticObstacles::createLine(pcl::PointXYZRGBL wallNode, pcl::PointXYZRGBL obstacleNode,
                                 const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints)
{
    float m;
    float b;

    std::vector<pcl::PointXYZRGBL> listOfNewWall;

    listOfNewWall.clear();

    m = (obstacleNode.x - wallNode.x) / (obstacleNode.y - wallNode.y);
    b = wallNode.x - m * wallNode.y;

    if (wallNode.y < obstacleNode.y)
    {
        for (float i = wallNode.y; i < obstacleNode.y; i = i + SCHRITTGROESSE)
        {
            pcl::PointXYZRGBL tmp;
            tmp.y = (i);
            tmp.x = (i)*m + b;
            tmp.z = 0;
            tmp.label = wallNode.label;
            listOfNewWall.push_back(tmp);
        }
    }
    else
    {
        for (float i = wallNode.y; i > obstacleNode.y; i = i - SCHRITTGROESSE)
        {
            pcl::PointXYZRGBL tmp;
            tmp.y = (i);
            tmp.x = (i)*m + b;
            tmp.z = 0;
            tmp.label = wallNode.label;
            listOfNewWall.push_back(tmp);
        }
    }

    for (auto& iter : wallPoints->points)
    {
        if (iter.label == 1 - wallNode.label)
        {
            listOfNewWall.push_back(iter);
        }
    }

    publishNewWall(&listOfNewWall);
}

void StaticObstacles::publishNewWall(std::vector<pcl::PointXYZRGBL>* clusters)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr msg(new pcl::PointCloud<pcl::PointXYZRGBL>);
    msg->header.frame_id = m_frame;

    for (auto& iter : *clusters)
    {
        msg->push_back(iter);
    }

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    m_newWall_publisher.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_static_obstacles");
    StaticObstacles staticObstacles;
    ros::spin();
    return EXIT_SUCCESS;
}