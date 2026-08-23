#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main() {
    return 0;
}

double getMotorStallTorque(const std::string& motorType) {
    static const std::unordered_map<std::string, double> motorTorques = {
        {"krakenX60FOC", 9.37},
        {"krakenX60", 7.09},
        {"falcon500", 4.69},
        {"neo", 3.28},
        {"neoVortex", 3.60}
    };

    auto it = motorTorques.find(motorType);
    if (it != motorTorques.end()) {
        return it->second;
    }
    return 7.09; // Default fallback
}

// 2. Parser function
trajopt::SwerveDrivetrain parseConfig(const std::string& file_name, double maxSpeed) {
    std::ifstream file(file_name);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + file_name);
    }

    json j;      // Added missing semicolon
    file >> j;   // Added missing semicolon

    trajopt::Translation2d frontLeft{
        j["flModuleX"].get<double>(), 
        j["flModuleY"].get<double>()
    };

    trajopt::Translation2d frontRight{
        j["frModuleX"].get<double>(), 
        j["frModuleY"].get<double>()
    };

    trajopt::Translation2d backLeft{
        j["blModuleX"].get<double>(), 
        j["blModuleY"].get<double>()
    };

    trajopt::Translation2d backRight{
        j["brModuleX"].get<double>(), 
        j["brModuleY"].get<double>()
    };

    double wheelRadius = j["driveWheelRadius"].get<double>();
    double maxWheelAngularVel = maxSpeed / wheelRadius;

    // Multiply motor stall torque by drive gearing to get wheel torque
    double motorStallTorque = getMotorStallTorque(j["driveMotorType"].get<std::string>());
    double driveGearing = j["driveGearing"].get<double>();
    double maxWheelTorque = motorStallTorque * driveGearing;

    // Wheel: radius (m), max angular velocity (rad/s), max torque at wheel (N*m)
    trajopt::Wheel wheel{wheelRadius, maxWheelAngularVel, maxWheelTorque};

    return trajopt::SwerveDrivetrain{
        j["robotMass"].get<double>(),
        j["robotMOI"].get<double>(),
        {frontLeft, frontRight, backLeft, backRight},
        wheel
    };
}