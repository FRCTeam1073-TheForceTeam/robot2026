// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LoaderTeleop.h"

LoaderTeleop::LoaderTeleop(std::shared_ptr<ShooterLoad> ShooterLoad) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterload{ShooterLoad} {
  {
  AddRequirements({m_shooterload.get()});
  }
}
// Called when the command is initially scheduled.
void LoaderTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LoaderTeleop::Execute() {
  m_shooterload->SetTargetLoadVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void LoaderTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool LoaderTeleop::IsFinished() {
  return false;
}
