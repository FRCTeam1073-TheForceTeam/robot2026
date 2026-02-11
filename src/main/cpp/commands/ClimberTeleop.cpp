// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberTeleop.h"

ClimberTeleop::ClimberTeleop(std::shared_ptr<Climber> climber) :
  m_climber{climber} {
  AddRequirements({m_climber.get()});
}

// Called when the command is initially scheduled.
void ClimberTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberTeleop::Execute() {
  m_climber->SetVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void ClimberTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberTeleop::IsFinished() {
  return false;
}
