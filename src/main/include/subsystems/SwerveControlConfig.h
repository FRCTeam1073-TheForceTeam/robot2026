
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/math.h>
#include <units/constants.h>

#include <cmath>



/**
 * This class is meant to hold the shared, common control configuration for the swerve drive modules.
 * There is only one shared instance of this class in a robot and all swerve modules share the same
 * controller configurations using a shared_ptr<> to it.
 */
class SwerveControlConfig {
    public:

    // Drive Constants:
    static constexpr units::length::meter_t DriveWheelDiameter = 0.1016_m;
    static constexpr auto DriveGearRatio = units::angle::turn_t(675.0)/ units::angle::turn_t(100.0);
    static constexpr auto DriveMetersPerMotorTurn = DriveWheelDiameter * units::constants::pi / (units::angle::turn_t(1.0) * DriveGearRatio);
    static constexpr units::current::ampere_t DriveCurrentLimit = 35.0_A;
    static constexpr units::voltage::volt_t DriveVoltageLimit = 8.0_V;
    static constexpr units::velocity::meters_per_second_t MaxModuleSpeed = 3.5_m/1.0_s;

    // Steer Constants:
    static constexpr auto SteerGearRatio = (units::angle::turn_t(150.0)/units::angle::turn_t(7.0));
    static constexpr units::current::ampere_t SteerCurrentLimit = 20.0_A;
    static constexpr units::voltage::volt_t SteerVoltageLimit = 8.0_V;

    // Controller Configurations:
    static ctre::phoenix6::configs::Slot0Configs GetDriveControlConfig();
    static ctre::phoenix6::configs::Slot0Configs GetSteerControlConfig();

    private:

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

};
