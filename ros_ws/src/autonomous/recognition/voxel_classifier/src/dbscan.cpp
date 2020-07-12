#include "dbscan.h"

int DBSCAN::run()
{
    int clusterID = 1;
    for (size_t i = 0; i < m_points->size(); i++)
    {
        if (m_points->at(i).label == UNCLASSIFIED)
        {
            if (expandCluster(m_points->at(i), clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }

    return 0;
}

int DBSCAN::expandCluster(Point_& point, int clusterID)
{
    std::vector<int> clusterSeeds = calculateCluster(point);

    if (clusterSeeds.size() < m_minPoints)
    {
        point.label = NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        std::vector<int>::iterator iterSeeds;
        for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points->at(*iterSeeds).label = clusterID;
            if (m_points->at(*iterSeeds).x == point.x && m_points->at(*iterSeeds).y == point.y &&
                m_points->at(*iterSeeds).z == point.z)
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

        for (vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
        {
            vector<int> clusterNeighors = calculateCluster(m_points->at(clusterSeeds[i]));

            if (clusterNeighors.size() >= m_minPoints)
            {
                vector<int>::iterator iterNeighors;
                for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
                {
                    if (m_points->at(*iterNeighors).label == UNCLASSIFIED || m_points->at(*iterNeighors).label == NOISE)
                    {
                        if (m_points->at(*iterNeighors).label == UNCLASSIFIED)
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points->at(*iterNeighors).label = clusterID;
                    }
                }
            }
        }

        return SUCCESS;
    }
}

vector<int> DBSCAN::calculateCluster(Point_ point)
{
    std::vector<int> clusterIndex;
    for (size_t i = 0; i < m_points->size(); i++)
    {
        if (calculateDistance(point, m_points->at(i)) <= m_epsilon)
        {
            clusterIndex.push_back(i);
        }
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(Point_ pointCore, Point_ pointTarget)
{
    return pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2) +
        pow(pointCore.z - pointTarget.z, 2) +
        m_color_weight *
        (pow(pointCore.r - pointTarget.r, 2) + pow(pointCore.g - pointTarget.g, 2) +
         pow(pointCore.b - pointTarget.b, 2));
}
