#pragma once
#include "geometric_math.h"
#include <cmath>
#include <vector>

class Circle
{
    Point m_center;
    double m_radius;

    public:
    Circle(Point center, double radius)
    {
        m_center = center;
        m_radius = radius;
    };

    Circle()
    {
        m_center = Point{ 0, 0 };
        m_radius = 0;
    };

    double getRadius()
    {
        return m_radius;
    }
    Point& getCenter()
    {
        return m_center;
    }
};