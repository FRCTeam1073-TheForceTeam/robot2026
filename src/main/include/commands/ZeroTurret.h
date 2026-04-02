// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Turret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

/**
 * Zero the turret by driving into the hard stop, seeing the current spike and resetting position.
 */
class ZeroTurret
    : public frc2::CommandHelper<frc2::Command, ZeroTurret> {
public:

  explicit ZeroTurret(std::shared_ptr<Turret> turret);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
private:
  std::shared_ptr<Turret> m_turret;
  units::torque::newton_meter_t limit;
};
