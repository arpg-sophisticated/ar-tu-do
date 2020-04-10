#include "imu_accumulator.h"

/**
 * @brief Starts the imu accumulator
 *
 * @param argc number of input arguments
 * @param argv input arguments
 * @return int exit code
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_accumulator");
    ImuAccumulator imu_accumulator;
    ros::spin();

    return EXIT_SUCCESS;
}