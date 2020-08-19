#pragma once

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <unordered_map>

constexpr const char* TOPIC_WALLS_ = "/obstacles/walls";
constexpr const char* TOPIC_OBSTACLES_ = "/obstacles/obstacles";
constexpr const char* TOPIC_NEWWALL_ = "/obstacles/newWall";
constexpr const float SCHRITTGROESSE = 0.2;
constexpr const float CLOSESTVOXELINTERVALL = 0.5;

class StaticObstacles
{
    private:
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;
    ros::Subscriber m_wall_subscriber;
    ros::Subscriber m_obstacles_subscriber;
    ros::Publisher m_newWall_publisher;
    std::string m_frame;

    std::unordered_map<int, std::vector<pcl::PointXYZRGBL>*> clustersUsed;

    int getFirstObstacle();
    std::pair<float, pcl::PointXYZRGBL> getClosestDistanceToWall(
        int WallSide, int obstacleID, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints);
    float getDistance(pcl::PointXYZRGBL obstacle, pcl::PointXYZRGBL wall);
    void createLine(pcl::PointXYZRGBL wallNode, pcl::PointXYZRGBL obstacleNode,
                    const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints);
    pcl::PointXYZRGBL getClosestWallVoxel(int WallID, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints);

    std::pair<int, pcl::PointXYZRGBL> getObstacleVoxel(int ObstacleID,
                                                       const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wallPoints);

    public:
    StaticObstacles();
    void wall_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& lidar);
    void obstacles_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& obstaclesPointcloud);

    void publishNewWall(std::vector<pcl::PointXYZRGBL>* clusters);
};
