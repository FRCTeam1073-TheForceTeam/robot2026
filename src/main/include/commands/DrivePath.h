// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/ProfiledPIDController.h>

#include <subsystems/DriveTrain.h>
#include <subsystems/Localizer.h>

#include <units/length.h>
#include <units/angle.h>

#include <choreo/Choreo.h>
#include <iostream>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DrivePath
    : public frc2::CommandHelper<frc2::Command, DrivePath> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  DrivePath(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    bool quit;

    units::meter_t distanceTolerance;
    units::radian_t angleTolerance;

    std::shared_ptr<Drivetrain> m_drivetrain;
    std::shared_ptr<Localizer> m_localizer;

    std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory;
    std::optional<choreo::SwerveSample> currentSample;

    frc::Pose2d robotPose;

    frc::PIDController xController;
    frc::PIDController yController;
    frc::PIDController thetaController;

    units::time::second_t currentTime;
    units::time::second_t startTime;
    units::time::second_t endTime;

    units::velocity::meters_per_second_t maxVelocity;
    units::angular_velocity::radians_per_second_t maxAngularVelocity;
    units::acceleration::meters_per_second_squared_t maxAcceleration;
    units::velocity::meters_per_second_t xVelocity;
    units::velocity::meters_per_second_t yVelocity;
    units::angular_velocity::radians_per_second_t thetaVelocity;


};
