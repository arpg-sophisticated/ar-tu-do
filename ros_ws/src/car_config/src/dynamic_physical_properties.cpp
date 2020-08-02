#include "dynamic_physical_properties.h"
#include "physical_properties.h"

DynamicPhysicalProperties::DynamicPhysicalProperties()
{
    m_dyn_cfg_server.setCallback([&](dynamic_physical_properties::dynamic_physical_propertiesConfig& cfg, uint32_t) {
        // add physical properties to the parameter server so that they can be used in other nodes and could be changed
        m_node_handle.setParam("/physical_properties/dynamic_friction", cfg.dynamic_friction);
        m_node_handle.setParam("/physical_properties/acceleration", 9.81 * cfg.dynamic_friction);
    });
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_physical_properties");
    DynamicPhysicalProperties dynamic_physical_properties;
    ros::spin();
    return EXIT_SUCCESS;
}