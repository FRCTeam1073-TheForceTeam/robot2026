
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Load.h"

Load::Load(std::shared_ptr<ShooterLoad> ShooterLoad) :
  m_shooterload{ShooterLoad} {
{
AddRequirements({m_shooterload.get()});
}
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Load::Initialize() {
  m_shooterload->SetTargetLoadVelocity(1.0_tps);
}

// Called repeatedly when this Command is scheduled to run
void Load::Execute() {
  m_shooterload->SetTargetLoadVelocity(1.0_tps);

}

// Called once the command ends or is interrupted.
void Load::End(bool interrupted) {
    m_shooterload->SetTargetLoadVelocity(0.0_tps);

}

// Returns true when the command should end.
bool Load::IsFinished() {
  return false;
}
