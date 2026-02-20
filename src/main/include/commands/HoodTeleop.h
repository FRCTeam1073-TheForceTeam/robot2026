// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterHood.h"
#include "subsystems/OI.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class HoodTeleop
    : public frc2::CommandHelper<frc2::Command, HoodTeleop> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  HoodTeleop(std::shared_ptr<ShooterHood> ShooterHood, std::shared_ptr<OI> OI);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
  private:
   std::shared_ptr<ShooterHood> m_shooterHood;
   std::shared_ptr<OI> m_OI;

   short level; // level that the hood is at (when multiplied by scale factor)
   const short maxLevel = 5; //TODO: change value after testing it
   const units::angle::radian_t ScaleFactor = 0.0131_rad; //TODO: change and test this value

   bool LeftBumper;
   bool RightBumper;
   bool LeftBumperPastState;
   bool RightBumperPastState;
};
