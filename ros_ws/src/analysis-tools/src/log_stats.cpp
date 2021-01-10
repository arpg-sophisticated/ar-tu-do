#include "log_stats.h"

LogStats::LogStats()
{
    m_controlled_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_CONTROLLED_DRIVE_PARAM, 1,
                                                         &LogStats::controlledDriveParametersCallback, this);
    m_max_speed_subscriber =
        m_node_handle.subscribe<std_msgs::Float64>(TOPIC_MAX_SPEED, 1, &LogStats::maxSpeedCallback, this);
    // m_gazebo_state_telemetry_subscriber =
    //    m_node_handle.subscribe<drive_msgs::gazebo_state_telemetry>(TOPIC_GAZEBO_STATE_TELEMETRY, 1,
    //                                                                &LogStats::gazeboStateTelemetryCallback, this);

    m_hud_speed_publisher = m_node_handle.advertise<std_msgs::Float32>("hud_speed_value", 1);
    m_hud_maxspeed_publisher = m_node_handle.advertise<std_msgs::Float32>("hud_maxspeed_value", 1);
    m_hud_rpm_publisher = m_node_handle.advertise<std_msgs::Float32>("hud_rpm_value", 1);
    m_hud_acceleration_publisher = m_node_handle.advertise<std_msgs::Float32>("hud_acceleration_value", 1);
    m_hud_angle_publisher = m_node_handle.advertise<std_msgs::Float32>("hud_angle_value", 1);
    m_hud_distance_publisher = m_node_handle.advertise<jsk_rviz_plugins::OverlayText>("hud_distance_value", 1);
    m_hud_clock_publisher = m_node_handle.advertise<jsk_rviz_plugins::OverlayText>("hud_clock_value", 1);
    m_hud_text_publisher = m_node_handle.advertise<jsk_rviz_plugins::OverlayText>("hud", 1);

    // parameters
    m_node_handle.param("/log_stats/prefix", m_log_prefix, (std::string) "ros");
    m_node_handle.param("/log_stats/length", m_mean_length, 100);
    m_node_handle.param("/log_stats/smooth", m_smooth_count, 5);
    m_node_handle.param("/log_stats/sim", m_is_simulation, false);
    m_node_handle.param("/log_stats/write", m_write_logfile, false);

    // some variables
    m_logentry = 0U;
    m_time_start = ros::Time::now();
    m_time_last = ros::Duration(0.0f);
    m_time_current = ros::Duration(0.0f);
    m_time_interval = ros::Duration(Config::LOG_INTERVAL);
    m_time_delta = ros::Duration(0.0f);

    // calculated data - speed
    m_speed_current = 0.0f;
    m_speed_last = 0.0f;
    m_speed_delta = 0.0f;
    m_speed_avg = 0.0f;
    m_speed_max = 0.0f;

    // calculated data - maximum speed from algorithm
    m_maxspeed_value = 0.0f;
    m_maxspeed_current = 0.0f;
    m_maxspeed_last = 0.0f;
    m_maxspeed_delta = 0.0f;
    m_maxspeed_avg = 0.0f;

    // calculated data - driven distance
    m_distance_current = 0.0f;
    m_distance_last = 0.0f;
    m_distance_delta = 0.0f;

    // calculated data - acceleration
    m_acceleration_current = 0.0f;
    m_acceleration_last = 0.0f;
    m_acceleration_min = 0.0f;
    m_acceleration_max = 0.0f;
    m_acceleration_delta = 0.0f;

    // calculated data - turn (0..1)
    m_turn_current = 0.0f;
    m_turn_last = 0.0f;
    m_turn_delta = 0.0f;

    // calculated data - angle derived from turn and MAX_STEERING_ANGLE
    m_angle_current = 0.0f;
    m_angle_last = 0.0f;
    m_angle_delta = 0.0f;

    // create logging path
    std::stringstream logpath;
    logpath << getLogPath();
    mkdir(logpath.str().c_str(), 0755);
    logpath << "/telemetry";
    mkdir(logpath.str().c_str(), 0755);
    logpath << "/" << m_log_prefix << "_" << getTimeString();
    mkdir(logpath.str().c_str(), 0755);

    // construct filenames
    std::stringstream datfile;
    datfile << logpath.str() << "/speed_over_time.dat";
    std::stringstream csvfile;
    csvfile << logpath.str() << "/speed_over_time.csv";
    m_filenamedat = datfile.str();
    m_filenamecsv = csvfile.str();
}

std::string LogStats::getLogPath()
{
    char logpath_delimiter = '/';
    std::string logpath_part = "";
    std::stringstream logpath;
    bool logpath_first = true;

    std::istringstream logpath_iss(ros::package::getPath("analysis-tools"));
    while (std::getline(logpath_iss, logpath_part, logpath_delimiter))
    {
        if (logpath_part.compare("ros_ws") == 0)
        {
            break;
        }
        else
        {
            if (!logpath_first)
            {
                logpath << "/";
            }
            else
            {
                logpath_first = false;
            }
            logpath << logpath_part;
        }
    }
    logpath << "/data";
    return logpath.str();
}

std::string LogStats::getTimeString()
{
    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", timeinfo);
    std::string timestring(buffer);
    return timestring;
}

void LogStats::controlledDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    // m_current_speed = parameters->velocity;
    // m_current_steering_angle = parameters->angle;
    m_time_last = m_time_current;
    if (m_is_simulation)
    {
        m_time_current = ros::Duration(ros::Time::now() - ros::Time(0.0f));
    }
    else
    {
        m_time_current = ros::Duration(ros::Time::now() - m_time_start);
    }
    m_time_delta += (m_time_current - m_time_last);
    if (m_time_delta >= m_time_interval)
    {
        // handle maxspeed
        m_maxspeed_last = m_maxspeed_current;
        m_maxspeed_current = m_maxspeed_value;
        m_maxspeed_delta = m_maxspeed_current - m_maxspeed_last;
        if (m_logentry > 0)
        {
            m_maxspeed_avg = (m_maxspeed_avg * (m_logentry - 1) + m_maxspeed_current) / m_logentry;
        }
        else
        {
            m_maxspeed_avg += m_maxspeed_current;
        }

        m_maxspeed_smooth.push_back(m_maxspeed_current);
        if (m_maxspeed_smooth.size() > (unsigned)m_smooth_count)
        {
            m_maxspeed_smooth.pop_front();
        }

        // get current speed and do calculations
        m_speed_last = m_speed_current;
        m_speed_current = parameters->velocity;
        m_speed_delta = m_speed_current - m_speed_last;

        if (m_logentry > 0)
        {
            m_speed_avg = (m_speed_avg * (m_logentry - 1) + m_speed_current) / m_logentry;
        }
        else
        {
            m_speed_avg += m_speed_current;
        }

        if (m_speed_current > m_speed_max)
        {
            m_speed_max = m_speed_current;
        }

        m_speed_maxtime.push_back(m_speed_current);
        if (m_speed_maxtime.size() > (unsigned)m_mean_length)
        {
            m_speed_maxtime.pop_front();
        }

        m_speed_avgtime.push_back(m_speed_current);
        if (m_speed_avgtime.size() > (unsigned)m_mean_length)
        {
            m_speed_avgtime.pop_front();
        }

        m_speed_smooth.push_back(m_speed_current);
        if (m_speed_smooth.size() > (unsigned)m_smooth_count)
        {
            m_speed_smooth.pop_front();
        }

        // get current turn and do calculations
        m_turn_last = m_turn_current;
        m_turn_current = parameters->angle;
        m_turn_delta = m_turn_current - m_turn_last;

        m_angle_last = m_angle_current;
        m_angle_current = m_turn_current * Config::MAX_STEERING_ANGLE;
        m_angle_delta = m_angle_current - m_angle_last;

        m_angle_smooth.push_back(m_angle_current);
        if (m_angle_smooth.size() > (unsigned)m_smooth_count)
        {
            m_angle_smooth.pop_front();
        }

        // calculate distances
        m_distance_delta = (m_time_delta.nsec * m_speed_current) / 1000000000;
        m_distance_last = m_distance_current;
        m_distance_current += m_distance_delta;

        // calculate acceleration
        m_acceleration_last = m_acceleration_current;
        if (m_time_delta.nsec > 0)
        {
            m_acceleration_current = m_speed_delta / ((double)m_time_delta.nsec / 1000000000);
        }
        m_acceleration_delta = m_acceleration_current - m_acceleration_last;

        if (m_acceleration_current > m_acceleration_max)
        {
            m_acceleration_max = m_acceleration_current;
        }

        if (m_acceleration_current < m_acceleration_min)
        {
            m_acceleration_min = m_acceleration_current;
        }

        m_acceleration_maxtime.push_back(m_acceleration_current);
        if (m_acceleration_maxtime.size() > (unsigned)m_mean_length)
        {
            m_acceleration_maxtime.pop_front();
        }

        m_acceleration_mintime.push_back(m_acceleration_current);
        if (m_acceleration_mintime.size() > (unsigned)m_mean_length)
        {
            m_acceleration_mintime.pop_front();
        }

        m_acceleration_smooth.push_back(m_acceleration_current);
        if (m_acceleration_smooth.size() > (unsigned)m_smooth_count)
        {
            m_acceleration_smooth.pop_front();
        }

        // write logfile
        if (m_write_logfile)
        {
            writeLogFile(" ", m_filenamedat);
            writeLogFile(";", m_filenamecsv);
        }

        // publish hud
        publishHud();

        // proceed
        m_time_delta = ros::Duration(0.0f);
        m_logentry++;
    }
}

void LogStats::publishHud()
{
    // hud publisher speed
    double m_speed_smooth_mean = 0.0f;
    for (double value : m_speed_smooth)
    {
        m_speed_smooth_mean += (value / m_speed_smooth.size());
    }
    std_msgs::Float32 speed_message;
    speed_message.data = m_speed_smooth_mean;
    m_hud_speed_publisher.publish(speed_message);

    // hud publisher maxspeed
    double m_maxspeed_smooth_mean = 0.0f;
    for (double value : m_maxspeed_smooth)
    {
        m_maxspeed_smooth_mean += (value / m_maxspeed_smooth.size());
    }
    std_msgs::Float32 maxspeed_message;
    maxspeed_message.data = m_maxspeed_smooth_mean;
    m_hud_maxspeed_publisher.publish(maxspeed_message);

    // hud publisher rpm
    double m_rpm_smooth_mean = 0.0f;
    for (double value : m_speed_smooth)
    {
        m_rpm_smooth_mean += (value / m_speed_smooth.size()) * Config::RPM_FACTOR;
    }
    std_msgs::Float32 rpm_message;
    rpm_message.data = m_rpm_smooth_mean;
    m_hud_rpm_publisher.publish(rpm_message);

    // hud publisher acceleration
    double m_acceleration_smooth_mean = 0.0f;
    for (double value : m_acceleration_smooth)
    {
        m_acceleration_smooth_mean += (value / m_acceleration_smooth.size());
    }
    std_msgs::Float32 acceleration_message;
    acceleration_message.data = m_acceleration_smooth_mean;
    m_hud_acceleration_publisher.publish(acceleration_message);

    // hud publisher angle
    double m_angle_smooth_mean = 0.0f;
    for (double value : m_angle_smooth)
    {
        m_angle_smooth_mean += (value / m_angle_smooth.size());
    }
    std_msgs::Float32 angle_message;
    angle_message.data = m_angle_smooth_mean * (-1);
    m_hud_angle_publisher.publish(angle_message);

    // hud publisher clock
    std::stringstream clock_message_text;
    int clock_tmsecs = std::round((double)m_time_current.nsec / 10000000);
    int clock_seconds = (int)m_time_current.sec % 60;
    int clock_minutes = ((int)m_time_current.sec / 60) % 60;
    int clock_hours = (int)m_time_current.sec / 3600;
    if (clock_hours < 10)
    {
        clock_message_text << "0";
    }
    clock_message_text << clock_hours << ":";
    if (clock_minutes < 10)
    {
        clock_message_text << "0";
    }
    clock_message_text << clock_minutes << ":";
    if (clock_seconds < 10)
    {
        clock_message_text << "0";
    }
    clock_message_text << clock_seconds << ".";
    if (clock_tmsecs < 10)
    {
        clock_message_text << "0";
    }
    clock_message_text << clock_tmsecs;
    jsk_rviz_plugins::OverlayText clock_message;
    std_msgs::ColorRGBA clock_color_fg;
    clock_color_fg.r = 0.09f;
    clock_color_fg.g = 1.0f;
    clock_color_fg.b = 0.9f;
    clock_color_fg.a = 1.0f;
    std_msgs::ColorRGBA clock_color_bg;
    clock_color_bg.r = 0.0f;
    clock_color_bg.g = 0.0f;
    clock_color_bg.b = 0.0f;
    clock_color_bg.a = 0.0f;
    clock_message.width = 300;
    clock_message.height = 30;
    clock_message.left = 39;
    clock_message.top = 20;
    clock_message.text_size = 24;
    clock_message.line_width = 3;
    clock_message.font = "Cousine";
    clock_message.text = clock_message_text.str();
    clock_message.fg_color = clock_color_fg;
    clock_message.bg_color = clock_color_bg;
    m_hud_clock_publisher.publish(clock_message);

    // hud publisher text
    std::deque<double>::iterator m_speed_maxtime_it_max =
        std::max_element(m_speed_maxtime.begin(), m_speed_maxtime.end());
    std::deque<double>::iterator m_acceleration_maxtime_it_max =
        std::max_element(m_acceleration_maxtime.begin(), m_acceleration_maxtime.end());
    std::deque<double>::iterator m_acceleration_mintime_it_min =
        std::min_element(m_acceleration_mintime.begin(), m_acceleration_mintime.end());

    double m_speed_avgtime_mean = 0.0f;
    for (double value : m_speed_avgtime)
    {
        m_speed_avgtime_mean += (value / m_speed_avgtime.size());
    }

    std::stringstream hud_message_text;

    hud_message_text << std::fixed << std::setprecision(Config::HUD_PRECISION) << "Time: " << clock_message_text.str()
                     << std::endl
                     << "Rcur: " << (m_speed_current >= 0 ? "+" : "") << m_speed_current * Config::RPM_FACTOR << " ^-1"
                     << std::endl
                     << "Rsmo: " << (m_speed_smooth_mean >= 0 ? "+" : "") << m_speed_smooth_mean * Config::RPM_FACTOR
                     << " ^-1" << std::endl
                     << "Vcur: " << (m_speed_current >= 0 ? "+" : "") << m_speed_current << " m/s" << std::endl
                     << "Vsmo: " << (m_speed_smooth_mean >= 0 ? "+" : "") << m_speed_smooth_mean << " m/s" << std::endl
                     << "Vavg: " << (m_speed_avg >= 0 ? "+" : "") << m_speed_avg << " m/s" << std::endl
                     << "Vav+: " << (m_speed_avgtime_mean >= 0 ? "+" : "") << m_speed_avgtime_mean << " m/s"
                     << std::endl
                     << "Vtop: " << (m_speed_max >= 0 ? "+" : "") << m_speed_max << " m/s" << std::endl
                     << "Vto+: " << (*m_speed_maxtime_it_max >= 0 ? "+" : "") << *m_speed_maxtime_it_max << " m/s"
                     << std::endl
                     << "Vmax: " << (m_maxspeed_current >= 0 ? "+" : "") << m_maxspeed_current << " m/s" << std::endl
                     << "Vmsm: " << (m_maxspeed_smooth_mean >= 0 ? "+" : "") << m_maxspeed_smooth_mean << " m/s"
                     << std::endl
                     << "Angl: " << (m_angle_current >= 0 ? "+" : "") << m_angle_current << "" << std::endl
                     << "Ansm: " << (m_angle_smooth_mean >= 0 ? "+" : "") << m_angle_smooth_mean << "" << std::endl
                     << "Turn: " << (m_turn_current >= 0 ? "+" : "") << m_turn_current << "" << std::endl
                     << "Acur: " << (m_acceleration_current >= 0 ? "+" : "") << m_acceleration_current << " m/s^2"
                     << std::endl
                     << "Asmo: " << (m_acceleration_smooth_mean >= 0 ? "+" : "") << m_acceleration_smooth_mean
                     << " m/s^2" << std::endl
                     << "Amin: " << (m_acceleration_min >= 0 ? "+" : "") << m_acceleration_min << " m/s^2" << std::endl
                     << "Amax: " << (m_acceleration_max >= 0 ? "+" : "") << m_acceleration_max << " m/s^2" << std::endl
                     << "Ato+: " << (*m_acceleration_maxtime_it_max >= 0 ? "+" : "") << *m_acceleration_maxtime_it_max
                     << " m/s^2" << std::endl
                     << "Ato-: " << (*m_acceleration_mintime_it_min >= 0 ? "+" : "") << *m_acceleration_mintime_it_min
                     << " m/s^2" << std::endl
                     << "Dist: " << (m_distance_current >= 0 ? "+" : "") << m_distance_current << " m" << std::endl
                     << "Iter:  " << std::max(0, ((int)(m_logentry - Config::LOGENTRY_OFFSET))) << "" << std::endl
                     << "AvgT:  " << m_mean_length << " (Vto+ Vav+ Ato+ Ato-)" << std::endl
                     << "AvgS:  " << m_smooth_count << " (Vsmo Asmo)" << std::endl;

    jsk_rviz_plugins::OverlayText hud_message;
    std_msgs::ColorRGBA hud_color_fg;
    hud_color_fg.r = 0.93f;
    hud_color_fg.g = 0.16f;
    hud_color_fg.b = 0.16f;
    hud_color_fg.a = 1.0f;
    std_msgs::ColorRGBA hud_color_bg;
    hud_color_bg.r = 0.0f;
    hud_color_bg.g = 0.0f;
    hud_color_bg.b = 0.0f;
    hud_color_bg.a = 0.0f;
    hud_message.width = 400;
    hud_message.height = 500;
    hud_message.top = 20;
    hud_message.left = 20;
    hud_message.text_size = 10;
    hud_message.line_width = 3;
    hud_message.font = "Cousine";
    hud_message.fg_color = hud_color_fg;
    hud_message.bg_color = hud_color_bg;
    hud_message.text = hud_message_text.str();
    m_hud_text_publisher.publish(hud_message);

    // hud publisher distance
    std::stringstream distance_message_text;
    distance_message_text << std::fixed << std::setprecision(2U) << m_distance_current << " m";
    jsk_rviz_plugins::OverlayText distance_message;
    std_msgs::ColorRGBA distance_color_fg;
    distance_color_fg.r = 0.09f;
    distance_color_fg.g = 1.0f;
    distance_color_fg.b = 0.9f;
    distance_color_fg.a = 1.0f;
    std_msgs::ColorRGBA distance_color_bg;
    distance_color_bg.r = 0.0f;
    distance_color_bg.g = 0.0f;
    distance_color_bg.b = 0.0f;
    distance_color_bg.a = 0.0f;
    distance_message.width = 300;
    distance_message.height = 30;
    if (m_distance_current > 10000)
    {
        distance_message.left = 58;
    }
    else if (m_distance_current >= 1000)
    {
        distance_message.left = 77;
    }
    else if (m_distance_current >= 100)
    {
        distance_message.left = 96;
    }
    else if (m_distance_current >= 10)
    {
        distance_message.left = 115;
    }
    else
    {
        distance_message.left = 134;
    }
    distance_message.top = 50;
    distance_message.text_size = 24;
    distance_message.line_width = 3;
    distance_message.font = "Cousine";
    distance_message.text = distance_message_text.str();
    distance_message.fg_color = distance_color_fg;
    distance_message.bg_color = distance_color_bg;
    m_hud_distance_publisher.publish(distance_message);
}

void LogStats::writeLogFile(std::string delimiter, std::string filename)
{
    // only run if offset is reached
    if (m_logentry >= Config::LOGENTRY_OFFSET)
    {

        // create file handlers
        std::ofstream filestream(filename, std::ios_base::app);

        // create logline
        std::stringstream logline;
        if (m_logentry == Config::LOGENTRY_OFFSET)
        {
            logline << "Datapoint" << delimiter << "AverageTime" << delimiter << "AverageSmooth" << delimiter << "Time"
                    << delimiter << "TimeDelta" << delimiter << "Speed" << delimiter << "SpeedDelta" << delimiter
                    << "SpeedAverage" << delimiter << "SpeedAverageTime" << delimiter << "SpeedMax" << delimiter
                    << "SpeedMaxTime" << delimiter << "Maxspeed" << delimiter << "MaxspeedDelta" << delimiter
                    << "MaxspeedAverage" << delimiter << "Angle" << delimiter << "AngleDelta" << delimiter
                    << "Acceleration" << delimiter << "AccelerationSmooth" << delimiter << "AccelerationDelta"
                    << delimiter << "AccelerationMin" << delimiter << "AccelerationMax" << delimiter
                    << "AccelerationMinTime" << delimiter << "AccelerationMaxTime" << delimiter << "Distance"
                    << delimiter << "DistanceDelta" << delimiter << "Turn" << delimiter << "TurnDelta" << delimiter
                    << "RPM" << std::endl;
        }

        std::deque<double>::iterator m_speed_maxtime_it_max =
            std::max_element(m_speed_maxtime.begin(), m_speed_maxtime.end());
        std::deque<double>::iterator m_acceleration_maxtime_it_max =
            std::max_element(m_acceleration_maxtime.begin(), m_acceleration_maxtime.end());
        std::deque<double>::iterator m_acceleration_mintime_it_min =
            std::min_element(m_acceleration_mintime.begin(), m_acceleration_mintime.end());

        double m_speed_avgtime_mean = 0.0f;
        for (double value : m_speed_avgtime)
        {
            m_speed_avgtime_mean += (value / m_speed_avgtime.size());
        }

        double m_acceleration_smooth_mean = 0.0f;
        for (double value : m_acceleration_smooth)
        {
            m_acceleration_smooth_mean += (value / m_acceleration_smooth.size());
        }

        logline << (m_logentry - Config::LOGENTRY_OFFSET) << delimiter << m_mean_length << delimiter << m_smooth_count
                << delimiter << m_time_current << delimiter << m_time_delta << delimiter << m_speed_current << delimiter
                << m_speed_delta << delimiter << m_speed_avg << delimiter << m_speed_avgtime_mean << delimiter
                << m_speed_max << delimiter << *m_speed_maxtime_it_max << delimiter << m_maxspeed_current << delimiter
                << m_maxspeed_delta << delimiter << m_maxspeed_avg << delimiter << m_angle_current << delimiter
                << m_angle_delta << delimiter << m_acceleration_current << delimiter << m_acceleration_smooth_mean
                << delimiter << m_acceleration_delta << delimiter << m_acceleration_min << delimiter
                << m_acceleration_max << delimiter << *m_acceleration_maxtime_it_max << delimiter
                << *m_acceleration_mintime_it_min << delimiter << m_distance_current << delimiter << m_distance_delta
                << delimiter << m_turn_current << delimiter << m_turn_delta << delimiter
                << m_speed_current * Config::RPM_FACTOR << std::endl;

        filestream << logline.str();
        filestream.close();
    }
}

void LogStats::maxSpeedCallback(const std_msgs::Float64::ConstPtr& max_speed)
{
    m_maxspeed_value = max_speed->data;
}

// void LogStats::gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr&
// gazebo_state_telemetry)
//{
// m_gazebo_wheel_speed = gazebo_state_telemetry->wheel_speed;
// m_gazebo_car_speed = gazebo_state_telemetry->car_speed;
// ... usw
//}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "log_stats");
    LogStats log_stats;
    ros::spin();
    return EXIT_SUCCESS;
}