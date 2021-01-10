#pragma once
#include <car_config/dynamic_physical_propertiesConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

class DynamicPhysicalProperties
{
    public:
    DynamicPhysicalProperties();

    private:
    ros::NodeHandle m_node_handle;
    dynamic_reconfigure::Server<dynamic_physical_properties::dynamic_physical_propertiesConfig> m_dyn_cfg_server;
};