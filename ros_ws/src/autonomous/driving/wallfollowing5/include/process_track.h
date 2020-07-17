#pragma once

// clang-format off
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "circle.h"
#include "circle_fit.h"
#include "geometric_math.h"
#include "rviz_geometry.h"
#include <functional>
#include <vector>
// clang-format on

typedef unsigned __int128 uint128_t;

enum CurveType
{
    CURVE_TYPE_STRAIGHT,
    CURVE_TYPE_RIGHT,
    CURVE_TYPE_LEFT
};

struct ProcessedTrack
{
    std::vector<Point> left_wall;
    std::vector<Point> right_wall;
    std::vector<Point> upper_wall;

    Circle left_circle;
    Circle right_circle;
    Circle upper_circle;

    CurveType curve_type;
    Point curve_entry;

    Point car_position;
};

class ProcessTrack
{
    RvizGeometry m_rviz_geometry;

    private:
    std::vector<Point> cropPointcloud(std::vector<Point>& pointcloud, float minimum_y);
    unsigned int findLeftRightBorder(std::vector<Point>& pointcloud);
    Point calcNearestPointToPoint(Point& point, std::vector<Point>& pointcloud);
    bool isCurveEntryInFront(Point& curve_entry_point, Point& lowest_point, double threshold);
    Point getCurveEntry(std::vector<Point>& wall);
    bool processTrack(ProcessedTrack* storage);
    bool wallIsStraight(std::vector<Point>& wall);

    public:
    bool processTrack(ProcessedTrack* storage, std::vector<Point>& pointcloud);
    bool processTrack(ProcessedTrack* storage, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& pointcloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pointcloud);
};