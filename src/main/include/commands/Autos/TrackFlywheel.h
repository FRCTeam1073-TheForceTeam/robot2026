// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Flywheel.h"
#include "subsystems/TargetFinder.h"
#include "utilities/ShooterTable.h"
#include <units/angular_velocity.h>

/*
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TrackFlywheel
    : public frc2::CommandHelper<frc2::Command, TrackFlywheel> {
 public:

  TrackFlywheel(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st, bool lookupTable = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


  private:

  std::shared_ptr<Flywheel> m_flywheel;
  std::shared_ptr<TargetFinder> m_tf;
  std::shared_ptr<ShooterTable> m_st;
  bool m_lookupTable;
};



















































































































































