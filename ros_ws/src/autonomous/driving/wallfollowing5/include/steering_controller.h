#include "car_config.h"
#include "config.h"
#include "geometric_math.h"
#include "physical_properties.h"
#include "pid_controller.h"
#include "process_track.h"
#include "rviz_geometry.h"
#include <cmath>
#include <drive_msgs/drive_param.h>

class SteeringController
{
    private:
    const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";

    double m_current_speed = 0;

    PIDController m_pid_controller;
    RvizGeometry m_rviz_geometry;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;

    public:
    SteeringController();

    double determineSteeringAngle(ProcessedTrack& processed_track, Config::PIDParams& pid_params,
                                  Config::SteeringParams& steering_params, Point& predicted_position,
                                  Point& target_position, double delta_time);
    void showSteeringAngle();

    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};