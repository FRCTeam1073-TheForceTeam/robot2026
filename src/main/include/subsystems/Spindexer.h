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
class Spindexer : public frc2::SubsystemBase {
 public:

  // CANBusID for the motor.
  static constexpr int SpindexerMotorId = 23;

  // Mechanism conversion constants for the subsystem:
  static constexpr auto TurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0);
  static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0);

  
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
      units::length::meter_t position;
      units::velocity::meters_per_second_t velocity;
      units::force::newton_t force;
  };


  // Commands may be modal (different command modes):
  // std::monostate is the "empty" command or "no command given".
  // Otherwise you can have two different types of commands.
  using Command = std::variant<std::monostate, units::velocity::meters_per_second_t, units::length::meter_t>;


  // Constructor for the subsystem.
  Spindexer();

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
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> GetVelocity();
  void SetTargetVelocity(units::angular_velocity::turns_per_second_t TargetVelocity);
  units::angular_velocity::turns_per_second_t GetTargetVelocity();

 private:


  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  units::angular_velocity::turns_per_second_t _targetVelocity;
  // Example TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _spindexerMotor;

  // CTRE hardware feedback signals:

  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _spindexerVelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _spindexerCurrentSig;


  // Example velocity and position controls:
  ctre::phoenix6::controls::VelocityVoltage _commandVelocityVoltage;  // Uses Slot0 gains.
  
  // Cached feedback:
  Feedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  //TODO: get gear ratio because EM doesnt have it yet
  double GearRatio = units::angle::turn_t(1)/units::angle::turn_t(1);
};
//hi if your reading this 