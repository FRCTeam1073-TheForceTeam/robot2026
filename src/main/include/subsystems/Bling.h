// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

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

  static constexpr int CANdleId = 30; //temp number
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
  };

  struct colorNumberLED{
    //signals::RGBWColor color;
    //TODO: un-comment the above line of code when it doesn't throw an error
    int startNum;
    int endNum;
  };

  using Command = std::variant<std::monostate, ctre::phoenix6::controls::SolidColor, Bling::colorNumberLED>;
  
  // Constructor for the subsystem.
  Bling();

  void Periodic() override;

  const Feedback& GetFeedback() const { return _feedback; }

  void SetCommand(Command cmd);

  frc2::CommandPtr blingWhite();

  frc2::CommandPtr blingPurple();
  

 private:

  ctre::phoenix6::hardware::CANdle _CANdle;

  units::voltage::volt_t _currentVoltage;

  static constexpr double alpha = 0.1; //TODO: tune alpha

  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  // Cached feedback:
  Feedback _feedback;

  Command  _command;
};
