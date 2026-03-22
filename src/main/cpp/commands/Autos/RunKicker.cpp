// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/RunKicker.h"

RunKicker::RunKicker(std::shared_ptr<Kicker>& kicker):
  m_kicker(kicker) {
    AddRequirements(m_kicker.get());
  }

// Called when the command is initially scheduled.`
void RunKicker::Initialize() {
  targetVelocity = 4.5_mps; 
}

// Called repeatedly when this Command is scheduled to run
void RunKicker::Execute() {
  m_kicker->SetCommand(targetVelocity);
}

// Called once the command ends or is interrupted.
void RunKicker::End(bool interrupted) {
  m_kicker->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool RunKicker::IsFinished() {
  return false;
}