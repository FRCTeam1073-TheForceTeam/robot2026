// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include "subsystems/Turret.h"
#include "subsystems/OI.h"
#include "subsystems/TargetFinder.h"
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

class TurretTeleop
    : public frc2::CommandHelper<frc2::Command, TurretTeleop> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */

  explicit TurretTeleop(std::shared_ptr<Turret>& turret, std::shared_ptr<OI>& oi,  std::shared_ptr<TargetFinder>& targetFinder);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private:
  
    std::shared_ptr<Turret> m_turret;
    std::shared_ptr<OI> m_OI;
    std::shared_ptr<TargetFinder> m_targetFinder;

    double lastError;

    double leftX;

    static constexpr units::angular_velocity::radians_per_second_t maximumRotationVelocity = 1.0_rad_per_s;//TODO: get maximum rotation velocity
  
    units::angle::radian_t targetAngle;
};
