#include "bruteforce_reward.h"

BruteforceReward::BruteforceReward()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &BruteforceReward::controlledDriveParametersCallback, this);
    
    std::string defaultresultsfile="/tmp/results.csv";
    m_node_handle.param("/bruteforce_reward/average", m_average, 1000);
    m_node_handle.param("/bruteforce_reward/bfresults", m_resultsfile, defaultresultsfile);
    m_logentry = 0U;
    m_time_start = ros::Time::now();
    m_time_last = ros::Duration(0.0f);
    m_time_current = ros::Duration(0.0f);
    m_time_interval = ros::Duration(Config::LOG_INTERVAL);
    m_time_delta = ros::Duration(0.0f);
    m_speed_current = 0.0f;
    std::stringstream logpath;
    logpath << m_resultsfile;
    remove( logpath.str().c_str() );
    std::ofstream filestream(logpath.str(), std::ios_base::app);
    std::stringstream logline;
    logline << "id;time;speed;avgcount;speed_avg;command" << std::endl;
    filestream << logline.str();
    filestream.close();
    m_command = "ok";
}

void BruteforceReward::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    std::stringstream logpath;
    logpath << m_resultsfile;
    std::ofstream filestream(logpath.str(), std::ios_base::app);
    
    m_time_last = m_time_current;
    m_time_current = ros::Duration(ros::Time::now() - m_time_start);
    m_time_delta += (m_time_current - m_time_last);
    m_time_current = ros::Duration(ros::Time::now() - m_time_start);
    
    if (m_time_delta >= m_time_interval)
    {
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
      logline << m_logentry << ";" << m_time_current << ";" << m_speed_current << ";" << m_average << ";" << m_speed_current_average_value << ";" << m_command << std::endl;
      filestream << logline.str();
      filestream.close();
      m_time_delta = ros::Duration(0.0f);
      m_logentry++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bruteforce_reward");
    BruteforceReward bruteforce_reward;
    ros::spin();
    return EXIT_SUCCESS;
}
