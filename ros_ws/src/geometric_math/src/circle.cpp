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
    int step_size = pointcloud.size() <= sample_count ? 1 : pointcloud.size() / sample_count;
    for (int i = 0; i < pointcloud.size(); i += step_size)
    {
        points.push_back(getClosestPoint(pointcloud[i]));
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

std::vector<Point> Circle::calcIntersections(Circle& circle)
{
    std::vector<Point> intersections;
    // calc vector between the two circles
    Point ab = Point{ circle.getCenter().x - getCenter().x, circle.getCenter().y - getCenter().y };
    double norm = std::sqrt(ab.x * ab.x + ab.y * ab.y);
    if (norm == 0)
    {
        // no distance between centers
        return intersections;
    }
    double x = (getRadius() * getRadius() + norm * norm - circle.getRadius() * circle.getRadius()) / (2 * norm);
    double y = getRadius() * getRadius() - x * x;
    if (y < 0)
    {
        // no intersection
        return intersections;
    }
    else if (y > 0)
    {
        y = std::sqrt(y);
    }
    // compute unit vectors
    Point ex = Point{ ab.x / norm, ab.y / norm };
    Point ey = Point{ -ex.y, ex.x };
    // compute intersection
    Point Q1 = Point{ getCenter().x + x * ex.x, getCenter().y + x * ex.y };
    if (y == 0)
    {
        // only one intersection
        intersections.push_back(Q1);
        return intersections;
    }
    Point Q2 = Point{ Q1.x - y * ey.x, Q1.y - y * ey.y };
    Q1 = Point{ Q1.x + y * ey.x, Q1.y + y * ey.y };
    intersections.push_back(Q1);
    intersections.push_back(Q2);
    return intersections;
}

std::vector<Point> Circle::calcTangents(Point& outside_point)
{
    Point center = getCenter();
    double radius = GeometricFunctions::distance(center, outside_point) / 2;
    Circle circle =
        Circle(Point{ (getCenter().x - outside_point.x) / 2, (getCenter().y - outside_point.y) / 2 }, radius);
    std::vector<Point> intersections = calcIntersections(circle);
    std::sort(intersections.begin(), intersections.end(), [](Point& a, Point& b) { return a.y > b.y; });
    return intersections;
}

double Circle::getDistance(Point& outside_point)
{
    return fabs(sqrt(((outside_point.x - m_center.x) * (outside_point.x - m_center.x)) +
                     ((outside_point.y - m_center.y) * (outside_point.y - m_center.y))) -
                m_radius);
}

bool Circle::pointIsInCircle(Point& p)
{
    return GeometricFunctions::distance(m_center, p) <= m_radius;
}

Circle Circle::determineCircle(Point& b, Point& c, Point& d)
{
    double temp = c.x + c.x + c.y * c.y;
    double bc = (b.x * b.x + b.y * b.y - temp) / 2.0;
    double cd = (temp - d.x * d.x - d.y * d.y) / 2.0;
    double det = (b.x - c.x) * (c.y - d.y) - (c.x - d.x) * (b.y - c.y);

    if (std::abs(det) < 1.0e-10)
        return Circle();

    // Center of circle
    Point center;
    center.x = (bc * (c.y - d.y) - cd * (b.y - c.y)) / det;
    center.y = ((b.x - c.x) * cd - (c.x - d.x) * bc) / det;

    double radius = GeometricFunctions::distance(center, b);

    return Circle(center, radius);
}