// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/current.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>

#include "SwerveControlConfig.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <string>

/**
 * There is one SwerveModule object instance for each hardware swerve module in the drivetrain.
 * 
 * This class encapsulates the hardware configuration and interface of each module behind a 
 * simple API with physical units that is used by the top-level Drivetrain class to manage
 * the whole drivetrain.
 */
class SwerveModule {
 public:

  /**
   * The collection of IDs that are associated with a specific swerve module and its number.
   */
  struct Ids {
    int number = -1;                  /// Every module has a unique numnber 0, through ...N (typically 4).
    int driveMotorId = -1;            /// Every module has a unique drive motor CAN ID in firmware
    int steerMotorId = -1;            /// Every module has a unique steer motor CAN ID in firmware
    int steerEncoderId = -1;          /// Every module has a unique encoder CAN ID in firmware
  };


  /**
   * Strongly typed swerve module detailed state feedback with units.
   */
  struct Feedback {
      units::time::second_t timeStamp = 0.0_s;
      units::angle::radian_t steeringAngle = 0.0_rad;
      units::angular_velocity::radians_per_second_t steeringVelocity = 0.0_rad_per_s;
      units::velocity::meters_per_second_t driveVelocity = 0.0_m/1.0_s;
      units::length::meter_t drivePosition = 0.0_m;
      units::current::ampere_t driveCurrent = 0.0_A; // Allows approximate effort computation and direction of force.
  };


  SwerveModule(Ids ids, frc::Translation2d location, const ctre::phoenix6::CANBus& canBus);

  /// @brief Is the configuration valid? 
  bool IsConfigurationValid() const;

  const Ids& GetConfiguration() const { return _ids;}

  /// @brief  Return the position of the module center within the robot.
  const frc::Translation2d& GetLocation() { return _location; }

  /// Configure the hardware for this module.
  bool ConfigureHardware();

  /// Return true if the hardware configuration succeeded.
  bool HardwareConfigured() const { return _hardwareConfigured; }

  /** Sample the hardware feedback information from the module, given the current timestamp which is store with the state.
   *  This avoids duplicating the sampling of time multiple times per update cycle.
   * 
   * This sampling implementation provides basic latency compensation for tighter control performance.
   * 
   */
  const Feedback& SampleFeedback(units::time::second_t now);

  /// Get the current cached swerve modulestate:
  const frc::SwerveModuleState& GetState() const { return _latestSwerveModuleState; }

  /// Get feedback as swerve module position: 
  const frc::SwerveModulePosition& GetPosition() const { return _latestSwerveModulePosition; }

  /// Send the command to the hardware and cache them.
  void SetCommand(frc::SwerveModuleState cmd, units::force::newton_t feedForward = 0_N);

  /// Set the drive brake mode:
  void SetDriveBrakeMode(bool brakes = true);

 private:

  // Helper function for configuring drive hardware.
  bool ConfigureDriveHardware();

  // Helper function for configuring steering hardware.
  bool ConfigureSteerHardware();

  // Did this module configure successfully?
  bool _hardwareConfigured;

  // Module-specific configuration:
  Ids _ids;
  frc::Translation2d _location; /// Every module has a unique position within the robot relative to robot center in hardware.


  // Hardware interfaces:
  ctre::phoenix6::hardware::TalonFX _steerMotor;
  ctre::phoenix6::hardware::TalonFX _driveMotor;
  ctre::phoenix6::hardware::CANcoder _steerEncoder;

  // Module-level controls:
  ctre::phoenix6::controls::VelocityVoltage _driveVelocityVoltage;
  ctre::phoenix6::controls::PositionVoltage _steerPositionVoltage;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _steerVelocitySig;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _steerPositionSig;

  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _driveVelocitySig;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _drivePositionSig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _driveCurrentSig;

  // Cached state read from hardware during most recent sample() operation:
  Feedback _latestFeedback;
  frc::SwerveModuleState _latestSwerveModuleState;
  frc::SwerveModulePosition _latestSwerveModulePosition;

  // Cached last command sent from drivetrain:
  frc::SwerveModuleState _targetState;
};
