#pragma once
#include "geometric_math.h"
#include <cmath>
#include <vector>

class Circle
{
    Point m_center;
    double m_radius;
    bool m_is_valid;

    public:
    Circle(Point center, double radius)
    {
        m_center = center;
        m_radius = radius;
        m_is_valid = true;
    };

    Circle(bool is_valid)
    {
        m_center = Point{ 0, 0 };
        m_radius = 0;
        m_is_valid = is_valid;
    };

    Circle()
    {
        m_center = Point{ 0, 0 };
        m_radius = 0;
        m_is_valid = true;
    };

    double getRadius()
    {
        return m_radius;
    }
    Point& getCenter()
    {
        return m_center;
    }

    bool isValid()
    {
        return m_is_valid;
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