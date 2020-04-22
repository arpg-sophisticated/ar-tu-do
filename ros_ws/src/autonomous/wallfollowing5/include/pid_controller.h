#pragma once

class PIDController
{
    double m_p;
    double m_i;
    double m_d;
    double m_anti_windup;

    double m_previous_error;
    double m_integral;

    public:
    PIDController(double p, double i, double d, double anti_windup=0.2)
    {
        m_p = p;
        m_i = i;
        m_d = d;
        m_anti_windup = anti_windup;
        m_integral = 0;
        m_previous_error = 0;
    }

    double updateAndGetCorrection(double error, double delta_time);
};