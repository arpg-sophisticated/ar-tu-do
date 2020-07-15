#include "pid_controller.h"

PIDController::PIDController()
{
    m_dyn_cfg_server.setCallback([&](pid::pidConfig& cfg, uint32_t) {
        m_p = cfg.pid_p;
        m_i = cfg.pid_i;
        m_d = cfg.pid_d;
        m_anti_windup = cfg.anti_windup;
    });
}

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