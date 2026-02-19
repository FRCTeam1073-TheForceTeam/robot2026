// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LoaderTeleop.h"

LoaderTeleop::LoaderTeleop(std::shared_ptr<Kicker> ShooterLoad, std::shared_ptr<OI> oi) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterload{ShooterLoad},
  m_OI{oi} {
  {
  AddRequirements({m_shooterload.get()});
  }
}
// Called when the command is initially scheduled.
void LoaderTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LoaderTeleop::Execute() {
  AButton = m_OI->GetOperatorAButton();
  if(AButton) {
    frc::SmartDashboard::PutBoolean("Kicker/IsSettingVelocity",true);
    m_shooterload->SetTargetLoadVelocity(1.65_mps);
  }
  else
  {
    frc::SmartDashboard::PutBoolean("Kicker/IsSettingVelocity",false);
    m_shooterload->SetTargetLoadVelocity(0.0_mps);
  }
}

// Called once the command ends or is interrupted.
void LoaderTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool LoaderTeleop::IsFinished() {
  return false;
}
