#ifndef ROBOT_CONFIG_PARSER
#define ROBOT_CONFIG_PARSER

#include <string>
#include <trajopt/drivetrain/swerve_drivetrain.hpp>

/**
 * @brief Retrieves the base stall torque (N*m) for a given motor type string.
 */
double getMotorStallTorque(const std::string& motorType);

/**
 * @brief Parses a PathPlanner settings JSON file into a TrajoptLib SwerveDrivetrain.
 * 
 * @param file_name Path to the settings JSON file.
 * @param maxSpeed Capped max linear velocity (m/s) for calculating wheel angular velocity limit.
 * @return trajopt::SwerveDrivetrain Constructed drivetrain object.
 */
trajopt::SwerveDrivetrain parseConfig(const std::string& file_name, double maxSpeed);

#endif // ROBOT_CONFIG_PARSER