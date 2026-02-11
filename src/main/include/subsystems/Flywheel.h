// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <variant>

#include <frc/filter/SlewRateLimiter.h>

class Flywheel : public frc2::SubsystemBase {
 public:

 struct FlywheelFeedback {
      units::velocity::meters_per_second_t velocity; // TODO: Add other stuff to feedback
      units::force::newton_t force;
  };

  using Command = std::variant<std::monostate, units::velocity::meters_per_second_t, units::length::meter_t>;

  Flywheel();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */


  static constexpr int LeadMotorId = 21; // TODO: Get motor id 
  static constexpr int FollowMotorId = 22;

  void Periodic() override;

  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> GetVelocity();

  units::angular_velocity::turns_per_second_t GetTargetVelocity();

  void SetVelocity(units::angular_velocity::turns_per_second_t Velocity);

  const FlywheelFeedback& GetFlywheelFeedback() const { return _feedback; }

  void SetCommand(Command cmd);

  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;



 private:

  const double GearRatio = units::angle::turn_t(1)/units::angle::turn_t(1); // TODO: Get gear ratio from EM

  

  // Mechanism conversion constants for the subsystem:
  static constexpr auto TurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0); // TODO: Get turns per meter
  static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0); // TODO: Get amps per newton

  
  //  TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _leadFlywheelMotor;
  ctre::phoenix6::hardware::TalonFX _followFlywheelMotor;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _FlywheelVelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _FlywheelCurrentSig;

  //  velocity and position controls:
  ctre::phoenix6::controls::VelocityVoltage _FlywheelVelocityVoltage;  // Uses Slot0 gains.
  
  // Cached feedback:
  FlywheelFeedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  // Set the motors target velocity
  units::angular_velocity::turns_per_second_t _TargetVelocity;

  frc::SlewRateLimiter<units::turns_per_second> limiter{0.5_tps / 1_s};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
