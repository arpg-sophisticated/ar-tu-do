#pragma once

#include "config.h"

class PIDController
{
    double m_p;
    double m_i;
    double m_d;
    double m_anti_windup;

    double m_previous_error = 0;
    double m_integral = 0;

    public:
    PIDController(double p, double i, double d, double anti_windup = 0.2)
    {
        m_p = p;
        m_i = i;
        m_d = d;
        m_anti_windup = anti_windup;
    }

    PIDController()
    {
        m_p = PID::P;
        m_i = PID::I;
        m_d = PID::D;
        m_anti_windup = PID::ANTI_WINDUP;
    }

    double updateAndGetCorrection(double error, double delta_time);
};