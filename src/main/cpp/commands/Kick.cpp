
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Kick.h"

Kick::Kick(std::shared_ptr<Kicker> Kick) :
  m_kicker{Kick} {
{
AddRequirements({m_kicker.get()});
}
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Kick::Initialize() {
  m_kicker->SetTargetLoadVelocity(1.0_tps);
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
