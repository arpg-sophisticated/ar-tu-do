#include "physical_properties.h"

float PhysicalProperties::getDynamicFriction()
{
    float friction;
    return ros::param::get("/physical_properties/dynamic_friction", friction) ? friction : 0;
}

float PhysicalProperties::getAcceleration()
{
    float acceleration;
    return ros::param::get("/physical_properties/acceleration", acceleration) ? acceleration : 0;
}