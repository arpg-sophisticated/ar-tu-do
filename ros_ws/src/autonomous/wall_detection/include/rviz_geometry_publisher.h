#pragma once

#include "std_msgs/ColorRGBA.h"
#include "voxel.h"
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

geometry_msgs::Point createPoint(double x, double y, double z);
std_msgs::ColorRGBA createColor(double r, double g, double b, double a);

class RvizGeometryPublisher {
public:
  RvizGeometryPublisher(ros::NodeHandle node_handle, const std::string &topic,
                        const std::string &frame);

  void drawLine(int id, geometry_msgs::Point point1,
                geometry_msgs::Point point2, std_msgs::ColorRGBA color,
                float width);

  void drawVoxels(int id, const PointCloud::ConstPtr &voxels, double width,
                  double height, double scoreFactor);
  void drawRect(int id, double x, double y, double width, double height,
                float intensity);

  void clearRects();

private:
  ros::Publisher m_marker_publisher;
  const std::string m_frame;
};
