// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climber.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimberTeleop
    : public frc2::CommandHelper<frc2::Command, ClimberTeleop> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param climber The subsystem used by this command.
   */

  explicit ClimberTeleop(std::shared_ptr<Climber> climber);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  std::shared_ptr<Climber> m_climber;
};
