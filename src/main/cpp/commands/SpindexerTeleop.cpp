// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpindexerTeleop.h"

SpindexerTeleop::SpindexerTeleop(std::shared_ptr<Spindexer> spindexer, std::shared_ptr<OI> OI) :
  m_spindexer{spindexer}, 
  m_OI{OI} {
  AddRequirements({m_spindexer.get(), m_OI.get()});
}

// Called when the command is initially scheduled.
void SpindexerTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SpindexerTeleop::Execute() {
  AButton = m_OI->GetDriverAButton();

  if (AButton){
    targetAngularVel = 1_rad_per_s;
  }
  else{
    targetAngularVel = 0_rad_per_s;
  }

  m_spindexer->SetTargetVelocity(targetAngularVel);

}

// Called once the command ends or is interrupted.
void SpindexerTeleop::End(bool interrupted) {
  targetAngularVel = 0_rad_per_s;
}

// Returns true when the command should end.
bool SpindexerTeleop::IsFinished() {
  return false;
}
