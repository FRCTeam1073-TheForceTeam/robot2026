// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/OI.h"
#include "subsystems/Bling.h"
#include <frc2/command/CommandPtr.h>

#include <frc/RobotController.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BlingTeleop
    : public frc2::CommandHelper<frc2::Command, BlingTeleop> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param bling The subsystem used by this command.
   */
  explicit BlingTeleop(std::shared_ptr<Bling>& Bling, std::shared_ptr<OI>& OI);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
   std::shared_ptr<Bling> m_bling;
   std::shared_ptr<OI> m_OI;

  units::voltage::volt_t _currentVoltage;

  static constexpr double alpha = 0.1; //TODO: tune alpha
};
