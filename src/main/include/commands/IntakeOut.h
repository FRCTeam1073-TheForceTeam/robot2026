// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

/**
 * Puts the intake out but can do it unsafely. Used for auto prep.
 */
class IntakeOut
    : public frc2::CommandHelper<frc2::Command, IntakeOut> {
public:

      /**
       * Create a IntakeOut Command
       * 
       * @param intake - the subsystem handle.
       * @param unsafe - Don't require the subsystem. Don't use except in auto prep!
       */
  explicit IntakeOut(std::shared_ptr<Intake> intake, bool unsafe = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
private:
  std::shared_ptr<Intake> m_intake;
};
