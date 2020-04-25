#include "circle.h"

std::vector<Point> Circle::createArray(std::vector<Point>& pointcloud, int sample_count)
{
    std::vector<Point> points;
    // double angle_step = (end_angle - start_angle) / sample_count;
    // double angle = start_angle;
    // for (int i = 0; i < sample_count; i++)
    // {
    //     Point point = { m_center.x + std::cos(angle) * m_radius, m_center.y + std::sin(angle) * m_radius };
    //     points.push_back(point);
    //     angle += angle_step;
    // }
    for (auto& point : pointcloud)
    {
        points.push_back(getClosestPoint(point));
    }
    return points;
}

double Circle::getAngle(Point& point)
{
    return std::atan2(point.x - m_center.x, point.y - m_center.y);
}

Point Circle::getClosestPoint(Point& point)
{
    double x = point.x - m_center.x;
    double y = point.y - m_center.y;
    double distance = std::sqrt(x * x + y * y);
    return Point{ m_center.x + x * m_radius / distance, m_center.y + y * m_radius / distance };
    // double angle = getAngle(point);
    // Point p;
    // p.x = m_center.x + std::sin(angle) * m_radius;
    // p.y = m_center.y + std::cos(angle) * m_radius;
    // return p;
}