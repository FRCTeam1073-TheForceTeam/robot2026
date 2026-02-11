// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoodTeleop.h"

HoodTeleop::HoodTeleop(std::shared_ptr<ShooterHood> ShooterHood) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_shooterHood.get()});
}

// Called when the command is initially scheduled.
void HoodTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HoodTeleop::Execute() {}

// Called once the command ends or is interrupted.
void HoodTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool HoodTeleop::IsFinished() {
  return false;
}
