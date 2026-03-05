#include "subsystems/SwerveControlConfig.h"

// https://motors.ctr-electronics.com/dyno/dynometer-testing/

ctre::phoenix6::configs::Slot0Configs SwerveControlConfig::GetDriveControlConfig() {
    ctre::phoenix6::configs::Slot0Configs config;
    config.kV = 0.12;   // kV for X60
    config.kP = 0.4;
    config.kI = 0.0;
    config.kD = 0.0;
    config.kA = 0.02;
    config.kS = 0.0;

    return config;
}

ctre::phoenix6::configs::Slot0Configs SwerveControlConfig::GetSteerControlConfig() {
    ctre::phoenix6::configs::Slot0Configs config;
    config.kV = 0.153;     // kV for X44
    config.kP = 10.0;
    config.kI = 0.4;
    config.kD = 0.02;
    config.kA = 0.0;
    config.kS = 0.05;

    return config;
}
