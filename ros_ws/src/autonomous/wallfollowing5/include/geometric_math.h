#pragma once

#include <cmath>

struct Point
{
    double x;
    double y;
};

namespace GeometricFunctions
{
    double distance(Point& a, Point& b);
};