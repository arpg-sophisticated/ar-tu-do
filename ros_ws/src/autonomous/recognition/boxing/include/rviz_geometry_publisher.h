#pragma once

#include "std_msgs/ColorRGBA.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::Point createPoint(double x, double y, double z);
std_msgs::ColorRGBA createColor(double r, double g, double b, double a);

class RvizGeometryPublisher
{
    public:
    RvizGeometryPublisher(ros::NodeHandle node_handle, const std::string& topic, const std::string& frame);

    void drawLine(int id, geometry_msgs::Point point1, geometry_msgs::Point point2, std_msgs::ColorRGBA color,
                  float width);

    void drawRect(int id, double x, double y, double width, double height, float intensity);

    void clearRects();

    private:
    ros::Publisher m_marker_publisher;
    const std::string m_frame;
};
