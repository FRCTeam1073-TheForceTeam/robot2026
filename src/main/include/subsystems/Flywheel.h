//SpeedUp Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>
#include <units/angle.h>
#include <units/constants.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <variant>

#include <frc/filter/SlewRateLimiter.h>

class Flywheel : public frc2::SubsystemBase {
 public:

  static constexpr int LeadMotorId = 21; 
  static constexpr int FollowMotorId = 22;

  static constexpr double GearRatio = units::angle::turn_t(1)/units::angle::turn_t(1);
  // Mechanism conversion constants for the subsystem:
  static constexpr units::meter_t wheelDiameter = units::inch_t(6.0);
  static constexpr auto TurnsPerMeter = units::turn_t(1) / (wheelDiameter * units::constants::pi); 
  static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0); // TODO: Get amps per newton

 struct Feedback {
      units::velocity::meters_per_second_t velocity; // TODO: Add other stuff to feedback
      units::force::newton_t force;
  };

  using Command = std::variant<std::monostate, units::velocity::meters_per_second_t>;

  Flywheel();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void Periodic() override;

  units::velocity::meters_per_second_t GetTargetVelocity();

  const Feedback& GetFeedback() const { return _feedback; }

  void SetCommand(Command cmd);

  frc2::CommandPtr SpinToSpeed(units::meters_per_second_t velocity);

 private:

   // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();
  
  //  TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _leadFlywheelMotor;
  ctre::phoenix6::hardware::TalonFX _followFlywheelMotor;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _flywheelVelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _flywheelCurrentSig;

  //  velocity and position controls:
  ctre::phoenix6::controls::VelocityVoltage _flywheelVelocityVoltage;  // Uses Slot0 gains.
  
  // Cached feedback:
  Feedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  frc::SlewRateLimiter<units::meters_per_second> _limiter;

    // Did we successfully configure the hardware?
  bool _hardwareConfigured;
};
