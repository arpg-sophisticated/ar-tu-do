#pragma once

namespace car_config
{
    /**
     * @brief Since it's not possible to use <cmath> in a constexpression, it's defined here again.
     */
    constexpr long double PI = 3.14159265358979323846;

    constexpr long double DEG_TO_RAD = PI / 180;

    /**
     * @brief The distance between the front and rear axes
     */
    constexpr long double WHEELBASE = 0.325;

    constexpr long double WHEEL_DIAMETER = 0.098;

    constexpr long double WHEEL_WIDTH = 0.042;

    constexpr long double FRONT_WHEEL_DISTANCE = 0.23;

    constexpr long double REAR_WHEEL_DISTANCE = 0.233;

    constexpr long double WHEEL_PERIMETER = WHEEL_DIAMETER * PI;

    constexpr long double TURNING_RADIUS = 0.605;

    /**
     * @brief Maximum revolutions per minute of the motor. Divide by TRANSMISSION to get the maximum wheel rpm
     * @unit 1/min
     */
    constexpr long double MAX_RPM_MECHANICAL = 60000;

    /**
     * @brief Number of electrical motor poles
     */
    constexpr long double MOTOR_POLES = 3;

    /**
     * @brief Maximum electrical revolutions per minute for use in the VESC
     * @unit 1/min
     */
    constexpr long double MAX_RPM_ELECTRICAL = MAX_RPM_MECHANICAL / MOTOR_POLES;

    /**
     * @brief Conversion factor from electrical revolutions per minute to meters per second
     * @unit m/s * minute = m
     */
    constexpr long double ERPM_TO_SPEED = WHEEL_PERIMETER * MOTOR_POLES / 60;

    /**
     * @brief Conversion factor from meters per second to electrical revolutions per minute
     * @unit s/ (m * minute) = 1/m
     */
    constexpr long double SPEED_TO_ERPM = 1 / ERPM_TO_SPEED;

    /**
     * @brief Conversion factor from mechanical revolutions per minute to meters per second
     * @unit m/s * minute = m
     */
    constexpr long double RPM_TO_SPEED = WHEEL_PERIMETER / 60;

    /**
     * @brief Position of the servo for normal wheel position
     */
    constexpr long double STEERING_TO_SERVO_OFFSET = 0.5;

    /**
     * @brief Conversion factor from steering angle to servo input
     * @unit 1/radian
     */
    constexpr long double STEERING_TO_SERVO_GAIN = -3 / PI;

    constexpr long double MAX_STEERING_ANGLE = 30 * DEG_TO_RAD;

    constexpr long double MIN_STEERING_ANGLE = -30 * DEG_TO_RAD;

    /**
     * @brief Gear transmission inside the differential of the car.
     * This is an estimate, the exact value is not known.
     */
    constexpr long double TRANSMISSION = 20;

    constexpr long double MAX_SERVO_POSITION = 1;

    /**
     * @brief Conversion factor from electrical revolutions per minute to radian to seconds
     * @unit radian/s * minute = radians
     */
    constexpr long double ERPM_TO_RAD_PER_SEC = MOTOR_POLES * 2 * PI / 60;
}; // namespace car_config
