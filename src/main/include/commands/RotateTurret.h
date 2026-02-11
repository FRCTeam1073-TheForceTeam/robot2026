// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include "subsystems/ShooterRotater.h"
#include "subsystems/OI.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <memory>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

//TODO: finish the command; it is not complete yet

class RotateTurret
    : public frc2::CommandHelper<frc2::Command, RotateTurret> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */

  explicit RotateTurret(std::shared_ptr<ShooterRotater> shooterRotater, std::shared_ptr<OI> oi);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private:
  
    std::shared_ptr<ShooterRotater> m_shooterRotater;
    std::shared_ptr<OI> m_OI;

    bool isAlignedToHub;

    double lastError;

    double leftX;

    static constexpr units::angular_velocity::radians_per_second_t maximumRotationVelocity = 1.0_rad_per_s;//TODO: get maximum rotation velocity
    
    units::angular_velocity::radians_per_second_t angularVel;
    units::angular_velocity::radians_per_second_t targetAngularVel;

    units::angle::radian_t targetPosition;
    units::angle::radian_t position;//zeroed position is up against the hard stop
    units::angle::radian_t maxPosition;
    units::angle::radian_t minPosition;

    units::force::newton_t torque;

};
