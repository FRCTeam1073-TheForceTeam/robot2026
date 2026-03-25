// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpindexerTeleop.h"

SpindexerTeleop::SpindexerTeleop(std::shared_ptr<Spindexer>& spindexer, std::shared_ptr<Kicker>& kicker, std::shared_ptr<OI>& OI) :
  m_spindexer(spindexer), 
  m_kicker(kicker),
  m_OI(OI) {
  AddRequirements(m_spindexer.get());
}

// Called when the command is initially scheduled.
void SpindexerTeleop::Initialize() {
  targetVelocity = 0_mps;
}

// Called repeatedly when this Command is scheduled to run
void SpindexerTeleop::Execute() {
  if (std::abs(m_OI->GetOperatorRightTrigger()) >= 0.1 && units::math::abs(m_kicker->GetFeedback().velocity) >= 3.0_mps) {
    // m_spindexer->SetCommand(7.15_mps);//was 5.75
    m_spindexer->SetCommand(6.9_mps);
  } else if (m_OI->GetOperatorBButton()) {
    m_spindexer->SetCommand(-2.0_mps);
  } else {
    m_spindexer->SetCommand(std::monostate());
  }
}

// Called once the command ends or is interrupted.
void SpindexerTeleop::End(bool interrupted) {
  targetVelocity = 0_mps;
  m_spindexer->SetCommand(std::monostate()); // Default no-command state.
}

// Returns true when the command should end.
bool SpindexerTeleop::IsFinished() {
  return false;
}
