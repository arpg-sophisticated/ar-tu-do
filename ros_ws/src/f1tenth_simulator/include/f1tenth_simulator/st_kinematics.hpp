#pragma once

#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/car_state.hpp"

namespace racecar_simulator
{

    class STKinematics
    {

        public:
        static CarState update(const CarState start, double accel, double steer_angle_vel, CarParams p, double dt);

        static CarState update_k(const CarState start, double accel, double steer_angle_vel, CarParams p, double dt);
    };

} // namespace racecar_simulator
