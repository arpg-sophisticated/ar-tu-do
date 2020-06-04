#pragma once

#include "circle.h"
#include "geometric_math.h"
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

constexpr const char* RVIZ_TOPIC = "/wallfollowing_visualization";
constexpr const char* RVIZ_FRAME = "laser";
constexpr const char* RVIZ_NAMESPACE = "wall_following";

struct ColorRGBA
{
    float r;
    float g;
    float b;
    float a;
};

class RvizGeometry
{
    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_marker_publisher;

    public:
    RvizGeometry();

    void showLineInRviz(int id, std::vector<Point>& points, ColorRGBA color, float line_width = 0.02);
    void showCircleInRviz(int id, Circle& circle, std::vector<Point>& wall, ColorRGBA color);
    void deleteMarker(int id);
};