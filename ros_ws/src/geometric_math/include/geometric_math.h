#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

struct Point
{
    float x;
    float y;
};

namespace GeometricFunctions
{
    const double PI = std::acos(-1);

    double distance(Point& a, Point& b);
    double distance(Point* a, Point* b);

    double toRadians(double degrees);
}; // namespace GeometricFunctions