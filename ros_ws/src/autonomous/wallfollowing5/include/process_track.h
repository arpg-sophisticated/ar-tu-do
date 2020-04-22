#pragma once

#include <vector>
#include "circle.h"

enum CurveType
{
    CURVE_TYPE_STRAIGHT;
    CURVE_TYPE_RIGHT; 
    CURVE_TYPE_LEFT;
};

struct ProcessedTrack
{
    std::vector<Point> left_wall;
    std::vector<Point> right_wall;
    std::vector<Point> upper_wall;

    Circle* left_circle;
    Circle* right_circle;
    Circle* upper_circle;

    CurveType curve_type;
    double remaining_distance;
};

class ProcessTrack
{
    public:
    ProcessTrack();

    int findLeftRightBorder(std::vector<Point>& pointcloud);
    ProcessedTrack& processTrack(std::vector<Point>& pointcloud);
};