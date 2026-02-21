// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberTeleop.h"

ClimberTeleop::ClimberTeleop(std::shared_ptr<Climber> climber, std::shared_ptr<OI> oi) :
  m_climber(climber),
  m_OI(oi) {

  AddRequirements({m_climber.get()});
}

// Called when the command is initially scheduled.
void ClimberTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberTeleop::Execute() {
  rightY = m_OI->GetOperatorRightY();
  if(abs(rightY) < 0.1) rightY = 0.0;

  vy = rightY*-0.4_mps;
  m_climber->SetCommand(vy);
}

// Called once the command ends or is interrupted.
void ClimberTeleop::End(bool interrupted) {
  m_climber->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool ClimberTeleop::IsFinished() {
  return false;
}
