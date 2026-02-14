
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Kick.h"

Kick::Kick(std::shared_ptr<Kicker> Kick, std::shared_ptr<OI> OI) :
  m_kicker{Kick},
  m_OI{OI} {
  targetAngularVel = 0_rad_per_s,
  angularVel = 0_rad_per_s,
  MenuButton = false,
  AddRequirements({m_kicker.get(), m_OI.get()});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Kick::Initialize() {
  MenuButton = m_OI->GetOperatorMenuButton();
  angularVel = m_kicker->GetVelocity();

  if (MenuButton){
    targetAngularVel = 1_rad_per_s; //TODO: change value after testing
  }
  else{
    targetAngularVel = 0_rad_per_s;
  }

  m_kicker->SetTargetLoadVelocity(targetAngularVel);
}

// Called repeatedly when this Command is scheduled to run
void Kick::Execute() {
  m_kicker->SetTargetLoadVelocity(1.0_tps);

}

// Called once the command ends or is interrupted.
void Kick::End(bool interrupted) {
    m_kicker->SetTargetLoadVelocity(0.0_tps);

}

// Returns true when the command should end.
bool Kick::IsFinished() {
  return false;
}
