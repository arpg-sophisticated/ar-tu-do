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

    std::vector<Point> createArray(std::vector<Point>& pointcloud, int sample_count = 50);
    double getAngle(Point& point);
    Point getClosestPoint(Point& point);
    std::vector<Point> calcIntersections(Circle& circle);
    std::vector<Point> calcTangents(Point& outside_point);
    double getDistance(Point& outside_point);
    bool pointIsInCircle(Point& p);

    static Circle determineCircle(Point& a, Point& b, Point& c);
};