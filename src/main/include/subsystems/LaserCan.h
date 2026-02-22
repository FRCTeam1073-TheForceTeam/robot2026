// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>
#include <grpl/LaserCan.h>

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
class LaserCan : public frc2::SubsystemBase {
 public:

  
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
    bool is_valid;
    units::length::meter_t range;
  };

  // Constructor for the subsystem.
  LaserCan();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   * This function samples and updates feedback from hardware, and sends target
   * command information to the hardware.
   */
  void Periodic() override;


  /// Access the latest feedback from the system. 
  const Feedback& GetFeedback() const { return _feedback; }

 private:

  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;
  
  // Cached feedback:
  Feedback _feedback;

  grpl::LaserCan _laserCAN;
};


