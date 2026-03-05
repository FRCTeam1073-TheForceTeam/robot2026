// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroHood.h"

ZeroHood::ZeroHood(std::shared_ptr<ShooterHood> shooterHood) :
  m_shooterHood(shooterHood) {
  AddRequirements({m_shooterHood.get()});
}

// Called when the command is initially scheduled.
void ZeroHood::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ZeroHood::Execute() {
  auto velocity = 1.00_rad_per_s;
  m_shooterHood->SetCommand(velocity);
}

// Called once the command ends or is interrupted.
void ZeroHood::End(bool interrupted) {
  m_shooterHood->Zero();
  m_shooterHood->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool ZeroHood::IsFinished() {
  if(m_shooterHood->GetFeedback().torque > 9_Nm) {
    return true;
  }
  return false;
}
