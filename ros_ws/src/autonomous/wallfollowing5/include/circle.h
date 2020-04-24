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

    double getRadius()
    {
        return m_radius;
    }
    Point getCenter()
    {
        return m_center;
    }

    std::vector<Point>& createArray(double start_angle, double end_angle, int sample_count = 50);
    double getAngle(Point point);
    Point getClosestPoint(Point& point);
    static Circle hyperFit(std::vector<Point> points);
};