// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

// Swerve kinematics support:
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
// "a wrapper around std:array that does compile time size checking"
#include <wpi/array.h>

// Hardware abstraction
#include <ctre/phoenix6/Pigeon2.hpp>

//Network tables import
#include <wpi/sendable/SendableBuilder.h>

#include "subsystems/SwerveModule.h"

class Drivetrain : public frc2::SubsystemBase {
 public:

  // CANBusID for the Pigeon2:
  static constexpr int PigeonId = 5;
  static const ctre::phoenix6::CANBus canBus;

  Drivetrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void InitSendable(wpi::SendableBuilder& builder) override;

    /// Set the debug mode
  void SetDebugMode(bool removeBug);

  units::angle::degree_t GetGyroHeadingDegrees();

  units::angle::radian_t GetGyroHeadingRadians();

  frc::Rotation2d GetGyroHeading();

  /// Get the pitch of the chassis:
  units::angle::degree_t GetPitch() const { return _pitchSig.GetValue(); }

  /// Get the roll of the chassis:
  units::angle::degree_t GetRoll() const { return _rollSig.GetValue(); }

  /// Return the chassis speeds for the drivetrain (in robot coordinates).
  const frc::ChassisSpeeds& GetChassisSpeeds() const { return _speeds; }

  /// Send the drivetrain a command (in robot coordinates).
  void SetChassisSpeeds(const frc::ChassisSpeeds& targetSpeeds) { _targetSpeeds = targetSpeeds; }

  /// Return the odomery pose of the drivetrain odometry (from most recent reset).
  const frc::Pose2d GetOdometry() const { return _odometry.GetPose(); }

  /// Reset the odometry to a specific pose on the field.
  void ResetOdometry(const frc::Pose2d pose);

  const frc::SwerveDriveKinematics<4U>& GetKinematics() { return _kinematics;}

  const wpi::array<frc::SwerveModulePosition, 4U>& GetSwerveModulePositions() const {return _swerveModulePositions;}
  /// Return the state of drivetrain brakes.  
  bool GetParkingBrake() const { return _parkingBrake; }

  /// Set the drive axis braking mode:
  void SetParkingBrake(bool brakeOn);

  /// Get average motor loading:
  units::force::newton_t GetAverageLoad() const;

  void ZeroHeading();

 private:

  /// Helper function to configure all hardware.
  bool ConfigureHardware();

  bool debug;

  // Hardware device for IMU sensor:
  ctre::phoenix6::hardware::Pigeon2 _imu;

  // Swerve module hardware:
  std::array<SwerveModule, 4> _swerveModules;

  // Module positions and states for feedback:
  wpi::array<frc::SwerveModulePosition, 4> _swerveModulePositions;
  wpi::array<frc::SwerveModuleState, 4> _swerveModuleStates;

  // Swerve drive kinematics.
  frc::SwerveDriveKinematics<4> _kinematics;

  // Swerve drive odometry.
  frc::SwerveDriveOdometry<4> _odometry;

  ctre::phoenix6::StatusSignal<units::angle::degree_t> _yawSig;
  ctre::phoenix6::StatusSignal<units::angle::degree_t> _pitchSig;
  ctre::phoenix6::StatusSignal<units::angle::degree_t> _rollSig;
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t> _yawRateSig;


  // Most recent chassis speeds: Computed in periodic.
  frc::ChassisSpeeds _speeds;

  // Most recent target chassis speeds: Processed in periodic.
  frc::ChassisSpeeds _targetSpeeds;

  // Was hardware configured correctly?
  bool _hardwareConfigured;


  // Overrides all control and forces the drivetrain into a locked "parking brake" condition.
  bool _parkingBrake;
};
