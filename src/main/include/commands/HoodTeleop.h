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
#include "utilities/ShooterTable.h"
#include "subsystems/TargetFinder.h"
#include "subsystems/ZoneFinder.h"

/**
 * HoodTeleop command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class HoodTeleop
    : public frc2::CommandHelper<frc2::Command, HoodTeleop> {
 public:

  HoodTeleop(std::shared_ptr<ShooterHood>& ShooterHood, std::shared_ptr<OI>& OI, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st, std::shared_ptr<ZoneFinder>& zone);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
  private:
   std::shared_ptr<ShooterHood> m_shooterHood;
   std::shared_ptr<OI> m_OI;
   std::shared_ptr<TargetFinder> m_hf;
   std::shared_ptr<ShooterTable> m_st;
   std::shared_ptr<ZoneFinder> m_zone;

   short level; // level that the hood is at (when multiplied by scale factor)
   const short maxLevel = 25; //TODO: change value after testing it
   const units::angle::radian_t ScaleFactor = 0.02_rad; //TODO: change and test this value

   bool LeftBumper;
   bool RightBumper;
   bool LeftBumperPastState;
   bool RightBumperPastState;
};
