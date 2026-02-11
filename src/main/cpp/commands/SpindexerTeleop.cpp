// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpindexerTeleop.h"

SpindexerTeleop::SpindexerTeleop(std::shared_ptr<Spindexer> spindexer) :
  m_spindexer{spindexer} {
  AddRequirements({m_spindexer.get()});
}

// Called when the command is initially scheduled.
void SpindexerTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SpindexerTeleop::Execute() {
  m_spindexer->SetTargetVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void SpindexerTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool SpindexerTeleop::IsFinished() {
  return false;
}
