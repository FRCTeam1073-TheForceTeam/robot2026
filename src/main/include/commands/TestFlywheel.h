// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Flywheel.h"
#include "subsystems/OI.h"

#include <units/angular_velocity.h>

/**
 * Flywheel Test command for shot tuning.
 */
class TestFlywheel
    : public frc2::CommandHelper<frc2::Command, TestFlywheel> {
 public:

  explicit TestFlywheel(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<OI>& oi);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  
 private:
  std::shared_ptr<Flywheel> m_flywheel;
  std::shared_ptr<OI> m_OI;

  bool lastDPadUp;
  bool lastDPadDown;

  int level;

  const units::velocity::meters_per_second_t ScaleFactor = 0.20_mps; // Speed per level.
  const int MaxLevel = 120;
  
};
