#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

struct Point
{
    float x;
    float y;

    bool is_valid()
    {
        return !std::isnan(x) && !std::isinf(x) && !std::isnan(y) && !std::isinf(y);
    }
};

struct Line
{
    Point start;
    Point end;

    float length()
    {
        return std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
    }
};

namespace GeometricFunctions
{
    const double PI = std::acos(-1);

    double distance(Point& a, Point& b);
    double distance(Point* a, Point* b);

    double toRadians(double degrees);

    double calcShortestDistanceToLine(Point& point, Line line);
}; // namespace GeometricFunctions