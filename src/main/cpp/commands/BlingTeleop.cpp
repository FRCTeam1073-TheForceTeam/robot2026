// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BlingTeleop.h"

BlingTeleop::BlingTeleop(std::shared_ptr<Bling>& Bling, std::shared_ptr<OI>& OI) :
  m_bling(Bling),
  m_OI(OI){
    AddRequirements({m_bling.get()});

  
}

// Called when the command is initially scheduled.
void BlingTeleop::Initialize() {
  m_bling->SetCommand("battery");
}

// Called repeatedly when this Command is scheduled to run
void BlingTeleop::Execute() {
  
}

// Called once the command ends or is interrupted.
void BlingTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool BlingTeleop::IsFinished() {
  return false;
}
