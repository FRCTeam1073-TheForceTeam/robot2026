// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Spindexer.h"
/**
 * Run the spindexer until interrupted. Does not finish on its own.
 */
class RunSpindexer
    : public frc2::CommandHelper<frc2::Command, RunSpindexer> {
 public:
 /**
   * Creates a new ExampleCommand.
   *
   * @param spindexer The subsystem used by this command.
   */
  
  explicit RunSpindexer(std::shared_ptr<Spindexer>& spindexer);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
   std::shared_ptr<Spindexer> m_spindexer;

   units::velocity::meters_per_second_t targetVelocity;
};
