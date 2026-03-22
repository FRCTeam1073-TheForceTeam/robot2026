// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterHood.h"

/**
 * Zero the hood by driving into hard stop, seeing current spike and resetting the zero position.
 */
class ZeroHood
    : public frc2::CommandHelper<frc2::Command, ZeroHood> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */

   /**
    * @param shooterHood
    */
  explicit ZeroHood(std::shared_ptr<ShooterHood> shooterHood);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    std::shared_ptr<ShooterHood> m_shooterHood;
};
