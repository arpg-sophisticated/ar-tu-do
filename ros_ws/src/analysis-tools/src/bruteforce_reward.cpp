#include "bruteforce_reward.h"

BruteforceReward::BruteforceReward()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &BruteforceReward::controlledDriveParametersCallback, this);
    
    m_node_handle.param("/bruteforce_reward/average", m_average, 1000);
    m_time_current = ros::Duration(0.0f);
    m_speed_current = 0.0f;
    std::stringstream logpath;
    std::istringstream logpath_iss(ros::package::getPath("analysis-tools"));
    logpath << logpath_iss.str() << "/../../bruteforce_reward.txt";
    remove( logpath.str().c_str() );
}

void BruteforceReward::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    std::stringstream logpath;
    std::istringstream logpath_iss(ros::package::getPath("analysis-tools"));
    logpath << logpath_iss.str() << "/../../bruteforce_reward.txt";
    std::ofstream filestream(logpath.str(), std::ios_base::app);
    
    m_time_current = ros::Duration(ros::Time::now() - ros::Time(0.0f));
    m_speed_current = parameters->velocity;

    m_speed_current_average.push_back(m_speed_current);
    if (m_speed_current_average.size() > (unsigned)m_average)
    {
        m_speed_current_average.pop_front();
    }
    double m_speed_current_average_value = 0.0f;
    for (double value : m_speed_current_average)
    {
        m_speed_current_average_value += (value / m_speed_current_average.size());
    }
    
    std::stringstream logline;
    logline << m_time_current << ";" << m_speed_current << ";" << m_average << ";" << m_speed_current_average_value << std::endl;
    filestream << logline.str();
    filestream.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bruteforce_reward");
    BruteforceReward bruteforce_reward;
    ros::spin();
    return EXIT_SUCCESS;
}