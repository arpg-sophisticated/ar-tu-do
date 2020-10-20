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
    // std::cout << "Value: " << val << " abs(val): " << fabsf(val) << std::endl;
    return fabsf(val) > threshold;
}

unsigned int ProcessTrack::findLeftRightBorder(std::vector<Point>& pointcloud,
                                               Config::ProcessingParams& processing_params)
{
    double max_distance = 0;
    unsigned int max_index = 0;
    unsigned int fallback_index = 0;

    unsigned int cropped_count = (double)pointcloud.size() / (double)processing_params.usable_laser_range *
        processing_params.usable_laser_range_wall_detection;
    unsigned int start_index = (pointcloud.size() - cropped_count) / 2;
    unsigned int end_index = cropped_count + (pointcloud.size() - cropped_count) / 2 - 1;

    for (unsigned int i = start_index; i < end_index; i++)
    {
        // maybe use pointcloud.at(...) with small performance penalty
        double distance = GeometricFunctions::distance(pointcloud[i], pointcloud[i + 1]);
        if (distance > max_distance && distance > MIN_SENSIBLE_TRACK_WIDTH)
        {
            if (distance < MAX_SENSIBLE_TRACK_WIDTH)
            {
                max_distance = distance;
                max_index = i + 1;
            }
            else
            {
                fallback_index = i + 1;
            }
        }
    }
    if (max_index == 0)
    {
        max_index = fallback_index;
    }
    if (pointcloud.size() > 1 && max_index > 0)
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

bool ProcessTrack::processTrack(ProcessedTrack* storage, Config::ProcessingParams& processing_params)
{
    storage->car_position = { 0, 0 };

    if (CircleFit::pointcloudIsValid(storage->left_wall) && CircleFit::pointcloudIsValid(storage->right_wall))
    {
        storage->left_circle = CircleFit::hyperFit(storage->left_wall);
        storage->right_circle = CircleFit::hyperFit(storage->right_wall);
        storage->right_valid = storage->left_valid = true;
    }
    else if (CircleFit::pointcloudIsValid(storage->left_wall) && !CircleFit::pointcloudIsValid(storage->right_wall))
    {
        storage->left_circle = CircleFit::hyperFit(storage->left_wall);
        int sign = storage->left_circle.pointIsInCircle(storage->car_position) ? -1 : 1;
        storage->right_circle = Circle(storage->left_circle.getCenter(), storage->left_circle.getRadius() + 4 * sign);
        std::cerr << "Right Wall invalid" << std::endl;
        storage->left_valid = true;
        storage->right_valid = false;
    }
    else if (CircleFit::pointcloudIsValid(storage->right_wall) && !CircleFit::pointcloudIsValid(storage->left_wall))
    {
        storage->right_circle = CircleFit::hyperFit(storage->right_wall);
        int sign = storage->right_circle.pointIsInCircle(storage->car_position) ? -1 : 1;
        storage->left_circle = Circle(storage->right_circle.getCenter(), storage->right_circle.getRadius() + 4 * sign);
        std::cerr << "Left Wall invalid" << std::endl;
        storage->left_valid = false;
        storage->right_valid = false;
    }
    else
    {
        std::cerr << "Right and Left Wall invalid" << std::endl;
        storage->left_valid = false;
        storage->right_valid = false;
        return false;
    }

    storage->curve_type = CURVE_TYPE_STRAIGHT;

    // With radius_proportions can be checked whether the car is approaching a curve or is on a straight part of the
    // track.
    double radius_proportions_left = storage->left_circle.getRadius() / storage->right_circle.getRadius();
    double radius_proportions_right = storage->right_circle.getRadius() / storage->left_circle.getRadius();
    if (radius_proportions_left > processing_params.radius_curve_entry_proportion &&
        storage->right_circle.getCenter().x < 0)
    {
        storage->curve_entry = getCurveEntry(storage->left_wall);
        Point nearest_point_to_car = calcNearestPointToPoint(storage->car_position, storage->left_wall);
        // if (isCurveEntryInFront(storage->curve_entry, nearest_point_to_car, 1))
        // {
        storage->upper_wall = cropPointcloud(storage->right_wall, storage->curve_entry.y - 1.5);
        storage->right_wall =
            cropPointcloud(storage->right_wall, [storage](Point& p) { return p.y <= storage->curve_entry.y; });
        storage->right_circle = CircleFit::hyperFit(storage->right_wall);
        storage->curve_type = CURVE_TYPE_LEFT;
        // }
    }
    else if (radius_proportions_right > processing_params.radius_curve_entry_proportion &&
             storage->left_circle.getCenter().x > 0)
    {
        storage->curve_entry = getCurveEntry(storage->right_wall);
        Point nearest_point_to_car = calcNearestPointToPoint(storage->car_position, storage->right_wall);
        // if (isCurveEntryInFront(storage->curve_entry, nearest_point_to_car, 1))
        // {
        storage->upper_wall = cropPointcloud(storage->left_wall, storage->curve_entry.y - 1.5);

        storage->left_wall =
            cropPointcloud(storage->left_wall, [storage](Point& p) { return p.y <= storage->curve_entry.y; });
        storage->left_circle = CircleFit::hyperFit(storage->left_wall);
        storage->curve_type = CURVE_TYPE_RIGHT;
        // }
    }

    if (storage->curve_type != CURVE_TYPE_STRAIGHT)
    {
        if (CircleFit::pointcloudIsValid(storage->upper_wall))
        {
            storage->upper_circle = CircleFit::hyperFit(storage->upper_wall);
            // storage->remaining_distance = GeometricFunctions::distance(storage->car_position, storage->curve_entry);
        }
        else
        {
            storage->curve_type = CURVE_TYPE_STRAIGHT;
            // std::cout << "upper wall ist not valid" << std::endl;
        }

        std::vector<Point> curve_entry_line = { Point{ -2, storage->curve_entry.y },
                                                Point{ 2, storage->curve_entry.y } };
        m_rviz_geometry.showLineInRviz(6, curve_entry_line, ColorRGBA{ 0.2, 0.5, 0.8, 1 });
        m_rviz_geometry.showCircleInRviz(7, storage->upper_circle, storage->upper_wall, ColorRGBA{ 0, 1, 1, 1 });
    }
    else
    {
        m_rviz_geometry.deleteMarker(6);
        m_rviz_geometry.deleteMarker(7);
    }

    m_rviz_geometry.showCircleInRviz(0, storage->left_circle, storage->left_wall, ColorRGBA{ 0.5, 1, 1, 1 });
    m_rviz_geometry.showCircleInRviz(1, storage->right_circle, storage->right_wall, ColorRGBA{ 0, 1, 1, 1 });

    return true;
}

bool ProcessTrack::processTrack(ProcessedTrack* storage, std::vector<Point>& pointcloud,
                                Config::ProcessingParams& processing_params)
{
    unsigned int wall_split_index = findLeftRightBorder(pointcloud, processing_params);
    for (unsigned int i = 0; i < wall_split_index; i++)
    {
        storage->right_wall.push_back(pointcloud[i]);
    }
    for (unsigned int i = wall_split_index; i < pointcloud.size(); i++)
    {
        storage->left_wall.push_back(pointcloud[i]);
    }

    return processTrack(storage, processing_params);
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
                                pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pointcloud,
                                Config::ProcessingParams& processing_params)
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
                        target = &storage->left_wall;
                    else if (voxel.label == 1)
                        target = &storage->right_wall;
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

    return processTrack(storage, processing_params);
}
