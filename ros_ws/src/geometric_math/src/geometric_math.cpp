#include "geometric_math.h"

double GeometricFunctions::distance(Point& a, Point& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double GeometricFunctions::distance(Point* a, Point* b)
{
    return std::sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

double GeometricFunctions::toRadians(double degrees)
{
    return degrees * PI / 180;
}

double GeometricFunctions::calcShortestDistanceToLine(Point& point, Line line)
{
    return std::abs((line.end.y - line.start.y) * point.x - (line.end.x - line.start.x) * point.y +
                    line.end.x * line.start.y - line.end.y * line.start.x) /
        line.length();
}