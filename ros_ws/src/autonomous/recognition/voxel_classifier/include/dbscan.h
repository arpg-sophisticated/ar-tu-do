// Von https://github.com/james-yoo/DBSCAN geklaut

#ifndef DBSCAN_H
#define DBSCAN_H

#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#define MAX_VALUE (4294967296 - 1)

#define UNCLASSIFIED (MAX_VALUE - 1)
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE (MAX_VALUE - 2)
#define SUCCESS 0
#define FAILURE -3

using namespace std;

/*typedef struct Point_
{
    float x, y, z, r, g, b; // X, Y, Z position
    int clusterID;          // clustered ID
} Point;*/

typedef pcl::PointXYZRGBL Point_;

class DBSCAN
{
    public:
    DBSCAN(unsigned int minPts, float eps, double color_weight, pcl::PointCloud<pcl::PointXYZRGBL>* points)
    {
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points->size();
        m_color_weight = color_weight;
    }
    ~DBSCAN()
    {
    }

    int run();
    vector<uint32_t> calculateCluster(Point_ point);
    int expandCluster(Point_& point, uint32_t clusterID);
    inline double calculateDistance(Point_ pointCore, Point_ pointTarget);

    int getTotalPointSize()
    {
        return m_pointSize;
    }
    int getMinimumClusterSize()
    {
        return m_minPoints;
    }
    int getEpsilonSize()
    {
        return m_epsilon;
    }

    private:
    pcl::PointCloud<pcl::PointXYZRGBL>* m_points;
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
    double m_color_weight;
};

#endif // DBSCAN_H
