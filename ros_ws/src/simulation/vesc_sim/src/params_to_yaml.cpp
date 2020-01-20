#include "car_config.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <ros/package.h>

int main(int argc, char** argv)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "speed_to_erpm_gain";
    out << YAML::Value << YAML::Precision(15) << car_config::SPEED_TO_ERPM;
    out << YAML::Key << "speed_to_erpm_offset";
    out << YAML::Value << 0;
    out << YAML::Key << "steering_angle_to_servo_gain";
    out << YAML::Value << YAML::Precision(15) << car_config::STEERING_TO_SERVO_GAIN;
    out << YAML::Key << "steering_angle_to_servo_offset";
    out << YAML::Value << YAML::Precision(15) << car_config::STEERING_TO_SERVO_OFFSET;
    out << YAML::Key << "wheelbase";
    out << YAML::Value << YAML::Precision(15) << car_config::WHEELBASE;
    out << YAML::EndMap;

    std::string package_path = ros::package::getPath("vesc_sim");
    std::ofstream fout(package_path + "/config/car_config.yaml");
    fout << out.c_str();
}
