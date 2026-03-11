// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/OI.h"
#include "subsystems/ShooterHood.h"

/**
 * Hood test command for shot tuning.
 */
class TestHood
    : public frc2::CommandHelper<frc2::Command, TestHood> {
 public:

  explicit TestHood(std::shared_ptr<ShooterHood>& hood, std::shared_ptr<OI>& oi);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    std::shared_ptr<ShooterHood> m_shooterHood;
    std::shared_ptr<OI> m_OI;

    int level;
    bool lastLeftBumper;
    bool lastRightBumper;

    const int MaxLevel = 32;
    const units::angle::radian_t ScaleFactor = 0.015625_rad;
};
