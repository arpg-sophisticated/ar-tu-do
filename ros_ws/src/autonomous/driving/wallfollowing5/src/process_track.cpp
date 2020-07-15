#include "process_track.h"
#include <boxing.h>
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <unordered_map>

std::vector<Point> ProcessTrack::cropPointcloud(std::vector<Point>& pointcloud, float minimum_y)
{
    std::vector<Point> cropped_pointcloud;
    for (auto& p : pointcloud)
    {
        if (p.y >= minimum_y)
        {
            cropped_pointcloud.push_back(p);
        }
    }
    return cropped_pointcloud;
}

Point ProcessTrack::calcNearestPointToPoint(Point& point, std::vector<Point>& pointcloud)
{
    double nearest_distance = 0;
    Point nearest_point;
    for (auto& p : pointcloud)
    {
        double distance = GeometricFunctions::distance(p, point);
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest_point = p;
        }
    }
    return nearest_point;
}

bool ProcessTrack::isCurveEntryInFront(Point& curve_entry_point, Point& lowest_point, double threshold)
{
    if (lowest_point.x - curve_entry_point.x == 0)
    {
        return true;
    }
    double val = (lowest_point.y - curve_entry_point.y) / (lowest_point.x - curve_entry_point.x);
    std::cout << "Value: " << val << " abs(val): " << fabsf(val) << std::endl;
    return fabsf(val) > threshold;
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
    if (pointcloud.size() > 1)
    {
        std::vector<Point> wall_seperation_points = { pointcloud[max_index - 1], pointcloud[max_index] };
        m_rviz_geometry.showLineInRviz(15, wall_seperation_points, ColorRGBA{ 0, 1, 0, 0.7 }, 0.005);
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

bool ProcessTrack::processTrack(ProcessedTrack* storage)
{
    if (!CircleFit::pointcloudIsValid(storage->left_wall))
    {
        std::cerr << "Left Wall invalid" << std::endl;
        return false;
    }
    if (!CircleFit::pointcloudIsValid(storage->right_wall))
    {
        std::cerr << "Right Wall invalid" << std::endl;
        return false;
    }
    storage->left_circle = CircleFit::hyperFit(storage->left_wall);
    storage->right_circle = CircleFit::hyperFit(storage->right_wall);

    storage->car_position = { 0, 0 };
    // With radius_proportions can be checked whether the car is approaching a curve or is on a straight part of the
    // track.

    storage->curve_type = CURVE_TYPE_STRAIGHT;

    double radius_proportions_left = storage->left_circle.getRadius() / storage->right_circle.getRadius();
    double radius_proportions_right = storage->right_circle.getRadius() / storage->left_circle.getRadius();
    if (radius_proportions_left > 1.2 && storage->right_circle.getCenter().x < 0)
    {
        storage->curve_entry = getCurveEntry(storage->left_wall);
        Point nearest_point_to_car = calcNearestPointToPoint(storage->car_position, storage->left_wall);
        std::cout << "curve_entry: " << storage->curve_entry.y << ", mod: " << storage->curve_entry.y - 1.5
                  << std::endl;
        if (isCurveEntryInFront(storage->curve_entry, nearest_point_to_car, 1))
        {
            storage->upper_wall = cropPointcloud(storage->right_wall, storage->curve_entry.y - 1.5);
            std::cout << "upper_wall size: " << storage->upper_wall.size() << std::endl;
            // storage->right_wall =
            //     cropPointcloud(storage->right_wall, [storage](Point& p) { return p.y <= storage->curve_entry.y; });
            // if len(right_wall) == 0:
            //     right_wall = upper_wall[0: 1]
            // try:
            // storage->right_circle = CircleFit::hyperFit(storage->right_wall);
            // except:
            //     print right_wall, upper_wall
            storage->curve_type = CURVE_TYPE_LEFT;
        }
    }
    else if (radius_proportions_right > 1.2 && storage->left_circle.getCenter().x > 0)
    {
        storage->curve_entry = getCurveEntry(storage->right_wall);
        Point nearest_point_to_car = calcNearestPointToPoint(storage->car_position, storage->right_wall);
        std::cout << "curve_entry: " << storage->curve_entry.y << ", mod: " << storage->curve_entry.y - 1.5
                  << std::endl;
        if (isCurveEntryInFront(storage->curve_entry, nearest_point_to_car, 1))
        {
            storage->upper_wall = cropPointcloud(storage->left_wall, storage->curve_entry.y - 1.5);
            std::cout << "upper_wall size: " << storage->upper_wall.size() << std::endl;
            // storage->left_wall =
            //     cropPointcloud(storage->left_wall, [storage](Point& p) { return p.y <= storage->curve_entry.y; });
            // if len(left_wall) == 0:
            //     left_wall = upper_wall[-2: -1]
            // try:
            // storage->left_circle = CircleFit::hyperFit(storage->left_wall);
            // except:
            //     print left_wall, upper_wall
            storage->curve_type = CURVE_TYPE_RIGHT;
        }
    }

    if (storage->curve_type != CURVE_TYPE_STRAIGHT)
    {
        double remaining_distance = storage->curve_entry.y;
        if (CircleFit::pointcloudIsValid(storage->upper_wall))
        {
            storage->upper_circle = CircleFit::hyperFit(storage->upper_wall);
        }
        else
        {
            storage->curve_type = CURVE_TYPE_STRAIGHT;
            std::cout << "upper wall ist not valid" << std::endl;
        }

        std::vector<Point> curve_entry_line = { Point{ -2, remaining_distance }, Point{ 2, remaining_distance } };
        m_rviz_geometry.showLineInRviz(6, curve_entry_line, ColorRGBA{ 0.2, 0.5, 0.8, 1 });
        m_rviz_geometry.showCircleInRviz(7, storage->upper_circle, storage->upper_wall, ColorRGBA{ 0, 1, 1, 1 });
    }
    else
    {
        m_rviz_geometry.deleteMarker(6);
        m_rviz_geometry.deleteMarker(7);
    }

    // std::cout << "l_c: " << storage->left_circle.getCenter().x << ", " << storage->left_circle.getCenter().y
    //           << " r_c: " << storage->right_circle.getCenter().x << std::endl;
    m_rviz_geometry.showCircleInRviz(0, storage->left_circle, storage->left_wall, ColorRGBA{ 0.5, 1, 1, 1 });
    m_rviz_geometry.showCircleInRviz(1, storage->right_circle, storage->right_wall, ColorRGBA{ 0, 1, 1, 1 });

    return true;
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

    return processTrack(storage);
}

bool ProcessTrack::wallIsStraight(std::vector<Point>& wall)
{
    if (wall.size() == 0)
        return false;
    double previous_x = wall[0].x;
    for (auto point : wall)
    {
        if (point.x != previous_x)
            return false;
        previous_x = point.x;
    }
    return true;
}

bool ProcessTrack::processTrack(ProcessedTrack* storage,
                                const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& wall_pointcloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pointcloud)
{
    double voxelSize = 0.15;
    if (!laser_pointcloud)
        return false;

    std::unordered_map<uint64_t, std::vector<Point>*> voxel_to_target_map;
    for (auto& point : *laser_pointcloud)
    {
        float x = point.x - remainderf(point.x, voxelSize);
        float y = point.y - remainderf(point.y, voxelSize);
        float z = point.z; // - remainderf(point.z, voxelSize);

        uint64_t voxel_id = Boxing::get_voxel_id(x, y, z);

        Point p = Point{ -point.y, point.x };
        if (!p.is_valid())
            continue;

        auto foundPair = voxel_to_target_map.find(voxel_id);
        bool found = foundPair != voxel_to_target_map.end();

        if (found)
        {
            std::vector<Point>* foundVector = (foundPair->second);
            foundVector->push_back(p);
        }
        else
        {
            for (auto& voxel : *wall_pointcloud)
            {
                if (voxel.x == x && voxel.y == y && voxel.z == z)
                {
                    std::vector<Point>* target;
                    if (voxel.label == 0)
                        target = &storage->right_wall;
                    else if (voxel.label == 1)
                        target = &storage->left_wall;
                    voxel_to_target_map[voxel_id] = target;
                    if (target)
                    {
                        target->push_back(p);
                    }
                    break;
                }
            }
        }
    }

    return processTrack(storage);
}