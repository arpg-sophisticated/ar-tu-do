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

namespace GeometricFunctions
{
    const double PI = std::acos(-1);

    double distance(Point& a, Point& b);
    double distance(Point* a, Point* b);

    double toRadians(double degrees);
}; // namespace GeometricFunctions