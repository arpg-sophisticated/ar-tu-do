#include "config.h"
#include "geometric_math.h"
#include "pid_controller.h"
#include "rviz_geometry.h"
#include <cmath>

class SteeringController
{
    PIDController m_pid_controller;
    RvizGeometry m_rviz_geometry;

    public:
    SteeringController()
    {
    }

    double determineSteeringAngle(Point& car_position, Point& predicted_position, Point& target_position,
                                  double delta_time);
    void showSteeringAngle();
};