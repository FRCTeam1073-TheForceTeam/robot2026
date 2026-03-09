// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/SetSpindexer.h"

SetSpindexer::SetSpindexer(std::shared_ptr<Spindexer>& spindexer):
  m_spindexer(spindexer) {
    AddRequirements(m_spindexer.get());
  }

// Called when the command is initially scheduled.`
void SetSpindexer::Initialize() {
  targetVelocity = 4.2_mps; 
}

// Called repeatedly when this Command is scheduled to run
void SetSpindexer::Execute() {
  m_spindexer->SetCommand(targetVelocity);
}

// Called once the command ends or is interrupted.
void SetSpindexer::End(bool interrupted) {
  m_spindexer->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool SetSpindexer::IsFinished() {
  return false;
}
