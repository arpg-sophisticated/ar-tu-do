#include "log_stats.h"

LogStats::LogStats()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &LogStats::controlledDriveParametersCallback, this);
}

void LogStats::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    m_current_speed = parameters->velocity;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "log_stats");
    LogStats log_stats;
    ros::spin();
    return EXIT_SUCCESS;
}