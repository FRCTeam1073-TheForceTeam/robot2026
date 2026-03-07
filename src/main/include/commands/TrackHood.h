// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterHood.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ShooterTable.h"
#include "subsystems/HubFinder.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TrackHood
    : public frc2::CommandHelper<frc2::Command, TrackHood> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  TrackHood(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<HubFinder>& hf, std::shared_ptr<ShooterTable>& st);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  std::shared_ptr<ShooterHood> m_shooterHood;
  std::shared_ptr<HubFinder> m_hf;
  std::shared_ptr<ShooterTable> m_st;
};
