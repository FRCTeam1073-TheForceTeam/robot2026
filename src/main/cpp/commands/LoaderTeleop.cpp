// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LoaderTeleop.h"

LoaderTeleop::LoaderTeleop(std::shared_ptr<Kicker> Kicker) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_kicker{Kicker} {
  {
  AddRequirements({m_kicker.get()});
  }
}
// Called when the command is initially scheduled.
void LoaderTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LoaderTeleop::Execute() {
  m_kicker->SetTargetLoadVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void LoaderTeleop::End(bool interrupted) {
  m_kicker->SetTargetLoadVelocity(0.0_tps);
}

// Returns true when the command should end.
bool LoaderTeleop::IsFinished() {
  return false;
}
