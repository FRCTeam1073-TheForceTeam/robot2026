// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <units/voltage.h>
#include <units/current.h>
#include <units/power.h>

#include <frc/PowerDistribution.h>

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
class PowerSubsystem : public frc2::SubsystemBase {
 public:


  // Constructor for the subsystem.
  PowerSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   * This function samples and updates feedback from hardware, and sends target
   * command information to the hardware.
   */
  void Periodic() override;

  /// @brief Return total amps we're using.
  /// @return Total current in amps.
  units::current::ampere_t GetTotalCurrant() const { return units::current::ampere_t(_powerDistribution.GetTotalCurrent()); }

  /// @brief Return system voltage.
  /// @return System voltage in volts.
  units::voltage::volt_t GetSystemVoltage() const { return units::voltage::volt_t(_powerDistribution.GetVoltage()); }

  
 private:

  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  // Example TalonFX motor interface.
  frc::PowerDistribution _powerDistribution;

};
