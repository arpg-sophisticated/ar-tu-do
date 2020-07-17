#include "pid_controller.h"

double PIDController::updateAndGetCorrection(double error, double delta_time)
{
    m_integral += error * delta_time;
    if (m_integral > m_anti_windup)
    {
        m_integral = m_anti_windup;
    }
    else if (m_integral < -m_anti_windup)
    {
        m_integral = -m_anti_windup;
    }

    double derivative = (error - m_previous_error) / delta_time;
    m_previous_error = error;
    return m_p * error + m_i * m_integral + m_d * derivative;
}

double PIDController::updateAndGetCorrection(double error, double delta_time, Config::PIDParams& pid_params)
{
    m_integral += error * delta_time;
    if (m_integral > pid_params.anti_windup)
    {
        m_integral = pid_params.anti_windup;
    }
    else if (m_integral < -pid_params.anti_windup)
    {
        m_integral = -pid_params.anti_windup;
    }

    double derivative = (error - m_previous_error) / delta_time;
    m_previous_error = error;
    return pid_params.p * error + pid_params.i * m_integral + pid_params.d * derivative;
}