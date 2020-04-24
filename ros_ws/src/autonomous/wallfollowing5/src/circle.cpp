#include "circle.h"

std::vector<Point>& Circle::createArray(double start_angle, double end_angle, int sample_count)
{
    std::vector<Point> points;
    double angle_step = std::abs(end_angle - start_angle) / sample_count;
    for (double angle = start_angle; angle <= end_angle; angle += angle_step)
    {
        Point point = { m_center.x + std::sin(angle) * m_radius, m_center.y + std::cos(angle) * m_radius };
        points.push_back(point);
    }
    return points;
}

double Circle::getAngle(Point point)
{
    return std::atan2(point.x - m_center.x, point.y - m_center.y);
}

Point Circle::getClosestPoint(Point& point)
{
    double x = point.x - m_center.x;
    double y = point.y - m_center.y;
    double distance = std::sqrt(x * x + y * y);
    return Point{ m_center.x + x * m_radius / distance, m_center.y + y * m_radius / distance };
}

Circle Circle::hyperFit(std::vector<Point> points)
{
    return Circle({ 0, 0 }, 1);
}