// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Kicker.h"
/**
 * Run the kicker until interrupted. Does not finish on its own.
 */
class RunKicker
    : public frc2::CommandHelper<frc2::Command, RunKicker> {
 public:
 /**
   * Creates a new ExampleCommand.
   *
   * @param kicker The subsystem used by this command.
   */
  
  explicit RunKicker(std::shared_ptr<Kicker>& kicker);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
   std::shared_ptr<Kicker> m_kicker;

   units::velocity::meters_per_second_t targetVelocity;
};
