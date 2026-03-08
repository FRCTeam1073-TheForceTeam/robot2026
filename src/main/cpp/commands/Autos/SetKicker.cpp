// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/SetKicker.h"

SetKicker::SetKicker(std::shared_ptr<Kicker>& kicker):
  m_kicker(kicker) {
    AddRequirements(m_kicker.get());
  }

// Called when the command is initially scheduled.`
void SetKicker::Initialize() {
  targetVelocity = 4.5_mps; 
}

// Called repeatedly when this Command is scheduled to run
void SetKicker::Execute() {
  m_kicker->SetCommand(targetVelocity);
}

// Called once the command ends or is interrupted.
void SetKicker::End(bool interrupted) {
  m_kicker->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool SetKicker::IsFinished() {
  return false;
}