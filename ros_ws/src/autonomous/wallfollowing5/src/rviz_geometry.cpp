#include "rviz_geometry.h"

RvizGeometry::RvizGeometry()
{
    m_marker_publisher = m_node_handle.advertise<visualization_msgs::Marker>(RVIZ_TOPIC, 1);
}

void RvizGeometry::showLineInRviz(int id, std::vector<Point>& points, ColorRGBA color, float line_width)
{
    visualization_msgs::Marker message;
    message.header.frame_id = RVIZ_FRAME;
    message.header.stamp = ros::Time::now();
    message.ns = RVIZ_NAMESPACE;
    message.action = visualization_msgs::Marker::ADD;
    message.pose.orientation.w = 1.0;

    message.id = id;
    message.type = visualization_msgs::Marker::LINE_STRIP;
    message.scale.x = line_width;
    message.color.r = color.r;
    message.color.g = color.g;
    message.color.b = color.b;
    message.color.a = color.a;

    for (auto& point : points)
    {
        geometry_msgs::Point p;
        p.y = -point.x;
        p.x = point.y;
        p.z = 0;

        message.points.push_back(p);
    }

    m_marker_publisher.publish(message);
}

void RvizGeometry::showCircleInRviz(int id, Circle& circle, std::vector<Point>& wall, ColorRGBA color)
{
    std::vector<Point> points = circle.createArray(wall, 100);
    showLineInRviz(id, points, color, 0.02);
}

void RvizGeometry::deleteMarker(int id)
{
    visualization_msgs::Marker message;
    message.header.frame_id = RVIZ_FRAME;
    message.header.stamp = ros::Time::now();
    message.ns = RVIZ_NAMESPACE;
    message.id = id;
    message.type = visualization_msgs::Marker::DELETE;
    m_marker_publisher.publish(message);
}