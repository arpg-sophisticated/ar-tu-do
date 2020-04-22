#pragma once
#include <cmath>
#include <vector>

struct Point
{
    double x;
    double y;
};

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

    double getRadius() { return m_radius; }
    Point getCenter() { return m_center; }

    std::vector<Point>& create_array(double start_angle, double end_angle, int sample_count = 50);
    double get_angle(Point point);
    Point& get_closest_point(Point& point);
    
};