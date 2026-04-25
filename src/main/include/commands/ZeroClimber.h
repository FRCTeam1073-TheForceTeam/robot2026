// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climber.h"

/**
 * Zero climber command.
 */
class ZeroClimber
    : public frc2::CommandHelper<frc2::Command, ZeroClimber> {
 public:

  /**
   * Creates a new ZeroClimber command.
   *
   * @param climber The subsystem used by this command.
   * @param unsafe If unsafe, don't require the subsystem. Use only for auto prep.
   */

  explicit ZeroClimber(std::shared_ptr<Climber> climber, bool unsafe = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    std::shared_ptr<Climber> m_climber;
};
