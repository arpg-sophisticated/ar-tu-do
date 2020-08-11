#pragma once

#include "circle.h"
#include "circle_fit.h"
#include "geometric_math.h"
#include <functional>
#include <vector>

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

    private:
    std::vector<Point> cropPointcloud(std::vector<Point>& pointcloud, std::function<bool(Point&)> select);
    unsigned int findLeftRightBorder(std::vector<Point>& pointcloud);
    Point getCurveEntry(std::vector<Point>& wall);

    public:
    bool processTrack(ProcessedTrack* storage, std::vector<Point>& pointcloud);
};