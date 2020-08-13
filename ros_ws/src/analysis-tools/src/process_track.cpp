#include "process_track.h"

std::vector<Point> ProcessTrack::cropPointcloud(std::vector<Point>& pointcloud, std::function<bool(Point&)> select)
{
    std::vector<Point> cropped_pointcloud;
    for (auto& p : pointcloud)
    {
        if (select(p))
        {
            cropped_pointcloud.push_back(p);
        }
    }
    return cropped_pointcloud;
}

unsigned int ProcessTrack::findLeftRightBorder(std::vector<Point>& pointcloud)
{
    double max_distance = 0;
    unsigned int max_index = 0;
    for (unsigned int i = 0; i < pointcloud.size() - 1; i++)
    {
        // maybe use pointcloud.at(...) with small performance penalty
        double distance = GeometricFunctions::distance(pointcloud[i], pointcloud[i + 1]);
        if (distance > max_distance)
        {
            max_distance = distance;
            max_index = i + 1;
        }
    }
    return max_index;
}

Point ProcessTrack::getCurveEntry(std::vector<Point>& wall)
{
    Point max_y_point = { 0, 0 };
    for (auto& point : wall)
    {
        if (point.y > max_y_point.y)
        {
            max_y_point = point;
        }
    }
    return max_y_point;
}

bool ProcessTrack::processTrack(ProcessedTrack* storage, std::vector<Point>& pointcloud)
{
    unsigned int wall_split_index = findLeftRightBorder(pointcloud);
    for (unsigned int i = 0; i < wall_split_index; i++)
    {
        storage->right_wall.push_back(pointcloud[i]);
    }
    for (unsigned int i = wall_split_index; i < pointcloud.size(); i++)
    {
        storage->left_wall.push_back(pointcloud[i]);
    }
    if (!CircleFit::pointcloudIsValid(storage->left_wall) || !CircleFit::pointcloudIsValid(storage->right_wall))
    {
        return false;
    }
    storage->left_circle = CircleFit::hyperFit(storage->left_wall);
    storage->right_circle = CircleFit::hyperFit(storage->right_wall);

    storage->car_position = { 0, 0 };
    // With radius_proportions can be checked whether the car is approaching a curve or is on a straight part of the
    // track.
    double radius_proportions_left = storage->left_circle.getRadius() / storage->right_circle.getRadius();
    double radius_proportions_right = storage->right_circle.getRadius() / storage->left_circle.getRadius();
    if (radius_proportions_left > 1.1 && storage->right_circle.getCenter().x < 0)
    {
        storage->curve_entry = getCurveEntry(storage->left_wall);
        storage->upper_wall =
            cropPointcloud(storage->right_wall, [storage](Point& p) { return p.y >= storage->curve_entry.y - 1.5; });
        storage->curve_type = CURVE_TYPE_LEFT;
    }
    else if (radius_proportions_right > 1.1 && storage->left_circle.getCenter().x > 0)
    {
        storage->curve_entry = getCurveEntry(storage->right_wall);
        storage->upper_wall =
            cropPointcloud(storage->left_wall, [storage](Point& p) { return p.y >= storage->curve_entry.y - 1.5; });
        storage->curve_type = CURVE_TYPE_RIGHT;
    }
    else
    {
        storage->curve_type = CURVE_TYPE_STRAIGHT;
    }

    if (!CircleFit::pointcloudIsValid(storage->left_wall) || !CircleFit::pointcloudIsValid(storage->right_wall))
    {
        return false;
    }

    if (storage->curve_type != CURVE_TYPE_STRAIGHT)
    {
        if (CircleFit::pointcloudIsValid(storage->upper_wall))
        {
            storage->upper_circle = CircleFit::hyperFit(storage->upper_wall);
        }
        else
        {
            storage->curve_type = CURVE_TYPE_STRAIGHT;
        }
    }

    return true;
}