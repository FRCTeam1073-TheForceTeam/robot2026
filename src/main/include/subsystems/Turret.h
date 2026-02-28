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
#include <frc/smartdashboard/SmartDashboard.h>

#include <variant>

#include <frc/filter/SlewRateLimiter.h>

/**
 * This example subsystem shows the basic pattern of any mechanism subsystem.
 * 
 * If configures some harwdare, provides feedback and accepts (modal) commands
 * for the subsystem. It handles feedback using signals (efficently),
 * and provides latency compensated feedabck. Lots of good practices for
 * a nicely behaved subsystem are included as example code to get started
 * on other subsystems.
 * 
 */
class Turret: public frc2::SubsystemBase {
 public:

  // CANBusID for the motor.
  static constexpr int RotaterMotorId = 25;
  static constexpr int encoderMotorId = 26;

  // Mechanism conversion constants for the subsystem:
  static constexpr double TurretToMotorTurns = (50.0 / 14.0) * (82.0 / 14.0); // Gear ratio.
  static constexpr auto AmpsPerNewtonMeter = units::current::ampere_t(10.0) / 1.0_Nm;

  
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
      units::angle::radian_t position;
      units::angular_velocity::radians_per_second_t velocity;
      units::torque::newton_meter_t torque;
  };


  // Commands may be modal (different command modes):
  // std::monostate is the "empty" command or "no command given".
  // Otherwise you can have two different types of commands.
  using Command = std::variant<std::monostate, units::angular_velocity::radians_per_second_t, units::angle::radian_t>;


  // Constructor for the subsystem.
  Turret();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   * This function samples and updates feedback from hardware, and sends target
   * command information to the hardware.
   */
  void Periodic() override;


  /// Access the latest feedback from the system. 
  const Feedback& GetFeedback() const { return _feedback; }

  /// Set the command for the system.
  void SetCommand(Command cmd);

  frc2::CommandPtr RotateToPos(units::radian_t pos);

 private:


  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  // Example TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _rotaterMotor;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _rotaterPositionSig;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _rotaterVelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _rotaterCurrentSig;


  // Example velocity and position controls:
  ctre::phoenix6::controls::PositionVoltage _commandPositionVoltage;  // Uses Slot0 gains.
  ctre::phoenix6::controls::VelocityVoltage _commandVelocityVoltage;

  // Cached feedback:
  Feedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  frc::SlewRateLimiter<units::radians> _limiter;
  
};
