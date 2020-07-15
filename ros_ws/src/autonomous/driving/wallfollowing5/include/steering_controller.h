#include "car_config.h"
#include "geometric_math.h"
#include "physical_properties.h"
#include "pid_controller.h"
#include "process_track.h"
#include "rviz_geometry.h"
#include <cmath>
#include <drive_msgs/drive_param.h>
#include <dynamic_reconfigure/server.h>
#include <wallfollowing5/steeringConfig.h>

class SteeringController
{
    private:
    const char* TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param";

    double m_current_speed = 0;
    double m_min_possible_steering_angle;
    double m_max_steering_angle;

    dynamic_reconfigure::Server<wallfollowing5::steeringConfig> m_dyn_cfg_server;

    PIDController m_pid_controller;
    RvizGeometry m_rviz_geometry;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_controlled_drive_parameters_subscriber;

    public:
    SteeringController();

    double determineSteeringAngle(ProcessedTrack& processed_track, Point& predicted_position, Point& target_position,
                                  double delta_time);
    void showSteeringAngle();

    void controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};