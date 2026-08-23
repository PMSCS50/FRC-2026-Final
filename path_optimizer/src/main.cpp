#include <iostream>
#include <exception>
#include <argparse/argparse.hpp>

#include "robot_config_parser.hpp"

using json = nlohmann::json;

int main() {
    try {
        trajopt::SwerveDrivetrain drivetrain = robot_config_parser::parseConfig("settings.json", 3.0);
        std::cout << "Successfully parsed robot configuration!\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}