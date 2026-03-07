// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/torque.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <variant>

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
class Intake : public frc2::SubsystemBase {
 public:

  // CANBusID for the motor.
  static constexpr int IntakeLeadId = 18;
  static constexpr int IntakeFollowId = 19;

  // Mechanism conversion constants for the subsystem:
  // Gear Ratio:
  static constexpr auto GearRatio = units::angle::turn_t(40) / units::angle::turn_t(1); // From new design.
  static constexpr auto AmpsPerNewtonMeter = units::current::ampere_t(10.0) / units::torque::newton_meter_t(1.0);
  units::angle::radian_t maxPosition = 0_rad;
  units::angle::radian_t minPosition = -2.13_rad;

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

\

  // Constructor for the subsystem.
  Intake();

  /** 
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   * This function samples and updates feedback from hardware, and sends target
   * command information to the hardware.
   */
  void Periodic() override;


  frc2::CommandPtr IntakeOut();

  frc2::CommandPtr IntakeIn();

  /// Access the latest feedback from the system. 
  const Feedback& GetFeedback() const { return _feedback; }

  /// Set the command for the system.
  void SetCommand(Command cmd);

  void Zero();

 private:


  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  // Example TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _leadMotor;
  ctre::phoenix6::hardware::TalonFX _followMotor;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _intakePositionSig;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _intakeVelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _intakeCurrentSig;


  // Example velocity and position controls:
  ctre::phoenix6::controls::PositionVoltage _commandPositionVoltage;  // Uses Slot0 gains.
  ctre::phoenix6::controls::VelocityVoltage _commandVelocityVoltage;  // Uses Slot1 gains.

  
  // Cached feedback:
  Feedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  frc::SlewRateLimiter<units::radians> _limiter;

};
