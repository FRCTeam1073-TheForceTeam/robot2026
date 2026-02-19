// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpindexerTeleop.h"

SpindexerTeleop::SpindexerTeleop(std::shared_ptr<Spindexer> spindexer, std::shared_ptr<OI> OI) :
  m_spindexer{spindexer}, 
  m_OI{OI} {
  spin = false; 
  AddRequirements(m_spindexer.get());
}

// Called when the command is initially scheduled.
void SpindexerTeleop::Initialize() {


}

// Called repeatedly when this Command is scheduled to run
void SpindexerTeleop::Execute() {  
  // if(m_OI->GetDriverAButton()) {
  //   spin = !spin;
  // }

  if (m_OI->GetOperatorAButton()){
    targetVelocity = 1.5_mps;
  }
  else if (m_OI->GetOperatorBButton()) {
    targetVelocity = -1.5_mps;
  }
  else{
    targetVelocity = 0_mps;
  }

  m_spindexer->SetCommand(targetVelocity);
  frc::SmartDashboard::PutBoolean("Spindexer/AButton", m_OI->GetOperatorAButton());
  
}

// Called once the command ends or is interrupted.
void SpindexerTeleop::End(bool interrupted) {
  targetVelocity = 0_mps;
}

// Returns true when the command should end.
bool SpindexerTeleop::IsFinished() {
  return false;
}
