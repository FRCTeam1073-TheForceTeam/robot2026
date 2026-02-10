#include "subsystems/SwerveControlConfig.h"


ctre::phoenix6::configs::Slot0Configs SwerveControlConfig::GetDriveControlConfig() {
    ctre::phoenix6::configs::Slot0Configs config;
    config.kV = 0.12;
    config.kP = 0.3;
    config.kI = 0.0;
    config.kD = 0.0;
    config.kA = 0.01;
    config.kS = 0.0;

    return config;
}

ctre::phoenix6::configs::Slot0Configs SwerveControlConfig::GetSteerControlConfig() {
    ctre::phoenix6::configs::Slot0Configs config;
    config.kV = 0.12;
    config.kP = 10.0;
    config.kI = 0.1;
    config.kD = 0.0;
    config.kA = 0.0;
    config.kS = 0.05;

    return config;
}


    // const double gearRatio = 6.75f;
    // const double wheelDiameterMeters = 0.1016f;
    // const double rotationsPerMeter = gearRatio/(wheelDiameterMeters * 1.06 * M_PI); // 1.06 is the measure correction factor while driving


    // const double radiansPerRotation = (150/7) / (2.0 * M_PI);
    // const double steerCurrentLimit = 20;
    // const double driveCurrentLimit = 35;
    // const double steerCurrentThreshold = 12;
    // const double driveCurrentThreshold = 22;
    // const double steerCurrentThresholdTime = 0.1;
    // const double driveCurrentThresholdTime = 0.25;
    // double steerP;
    // double steerI;
    // double steerD;
    // double steerV;
    // double driveP;
    // double driveI;
    // double driveD;
    // double driveV;
    // double driveA;
    // const double driveMaxIntegrator = 400.0;
    // const double steerMaxIntegrator = 400.0;