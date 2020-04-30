#include "rviz_geometry_publisher.h"

RvizGeometryPublisher::RvizGeometryPublisher(ros::NodeHandle node_handle, const std::string& topic,
                                             const std::string& frame)
    : m_frame(frame)
{
    this->m_marker_publisher = node_handle.advertise<visualization_msgs::Marker>(topic, 10);
}

void RvizGeometryPublisher::drawLine(int id, geometry_msgs::Point point1, geometry_msgs::Point point2,
                                     std_msgs::ColorRGBA color, float width)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = this->m_frame;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = width;
    line_list.color = color;

    line_list.points.push_back(point1);
    line_list.points.push_back(point2);

    this->m_marker_publisher.publish(line_list);
}

void RvizGeometryPublisher::drawVoxels(int id, std::vector<Voxel> voxels, double width, double height,
                                       double scoreFactor)
{
    visualization_msgs::Marker rects;
    rects.type = visualization_msgs::Marker::CUBE_LIST;
    rects.header.frame_id = this->m_frame;
    rects.header.stamp = ros::Time::now();

    rects.id = id;
    rects.ns = "voxels";

    for (auto voxel : voxels)
    {
        rects.points.push_back(createPoint(voxel.x, voxel.y, 0.0f));
        std::cout << voxel.x << " t " << voxel.y << " t " << voxel.get_score() << "\n";
        float score = voxel.get_score() * scoreFactor;
        if (score > 1.0f)
            score = 1.0f;
        std_msgs::ColorRGBA color;
        switch (voxel.clusterID)
        {
            case 1:
                color.r = 1.0f;
                color.g = 0.0f;
                color.b = 0.0f;
                color.a = 1.0f;
                break;
            case 2:
                color.r = 0.0f;
                color.g = 1.0f;
                color.b = 0.0f;
                color.a = 1.0f;
                break;
            case 3:
                color.r = 0.0f;
                color.g = 0.0f;
                color.b = 1.0f;
                color.a = 1.0f;
                break;

            default:
                color.r = 1.0f;
                color.g = 1.0f;
                color.b = 1.0f;
                color.a = 1.0f;
                break;
        }

        rects.colors.push_back(color);
    }

    rects.pose.orientation.w = 1;

    rects.scale.x = width;
    rects.scale.y = height;
    rects.scale.z = 0.02f;

    rects.action = visualization_msgs::Marker::ADD;

    this->m_marker_publisher.publish(rects);
}

void RvizGeometryPublisher::drawRect(int id, double x, double y, double width, double height, float intensity = 1.0f)
{
    visualization_msgs::Marker rect;
    rect.type = visualization_msgs::Marker::CUBE;
    rect.header.frame_id = this->m_frame;
    rect.header.stamp = ros::Time::now();

    rect.id = id;
    rect.ns = "rects";

    rect.action = visualization_msgs::Marker::ADD;

    rect.pose.position.x = x;
    rect.pose.position.y = y;
    rect.pose.position.z = 0;
    rect.pose.orientation.x = 0;
    rect.pose.orientation.y = 0;
    rect.pose.orientation.z = 0;
    rect.pose.orientation.w = 1.0;

    rect.scale.x = width;
    rect.scale.y = height;
    rect.scale.z = 0.01f;

    rect.color.r = 1.0f;
    rect.color.g = 0.0f;
    rect.color.b = 0.0f;
    rect.color.a = intensity;

    this->m_marker_publisher.publish(rect);
}

void RvizGeometryPublisher::clearRects()
{
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    clear.ns = "rects";

    this->m_marker_publisher.publish(clear);
}

geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::ColorRGBA createColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}