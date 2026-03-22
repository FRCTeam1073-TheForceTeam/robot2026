// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

/**
 * Zero the intake by driving into the hard stop, seeing current spike and resetting position.
 */
class ZeroIntake
    : public frc2::CommandHelper<frc2::Command, ZeroIntake> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  ZeroIntake(std::shared_ptr<Intake> intake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
      std::shared_ptr<Intake> m_intake;
};
