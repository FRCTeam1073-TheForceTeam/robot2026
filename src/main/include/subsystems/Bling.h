// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANdle.hpp>

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
class Bling : public frc2::SubsystemBase {
 public:

  static constexpr int CANdleId = 37; //temp number
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
  };

  using Command = std::variant<int>;
  
  // Constructor for the subsystem.
  Bling();

  void Periodic() override;

  const Feedback& GetFeedback() const { return _feedback; }

  void SetCommand(Command cmd);
  

 private:

  ctre::phoenix6::hardware::CANdle _CANdle;

  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  // Cached feedback:
  Feedback _feedback;

  Command  _command;
};
