#pragma once

#include <cmath>

struct Point
{
    double x;
    double y;
};

namespace GeometricFunctions
{
    const double PI = std::acos(-1);

    double distance(Point& a, Point& b);

    double toRadians(double degrees);
}; // namespace GeometricFunctions