// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Spindexer.h"
#include "subsystems/OI.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SpindexerTeleop
    : public frc2::CommandHelper<frc2::Command, SpindexerTeleop> {
 public:
 /**
   * Creates a new ExampleCommand.
   *
   * @param spindexer The subsystem used by this command.
   */

  explicit SpindexerTeleop(std::shared_ptr<Spindexer> spindexer, std::shared_ptr<OI> OI);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
   std::shared_ptr<Spindexer> m_spindexer;
   std::shared_ptr<OI> m_OI;

    bool spin;
    
    units::velocity::meters_per_second_t velocity;
    units::velocity::meters_per_second_t targetVelocity;


};
