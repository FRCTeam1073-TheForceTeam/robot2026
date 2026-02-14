// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoodTeleop.h"

HoodTeleop::HoodTeleop(std::shared_ptr<ShooterHood> ShooterHood, std::shared_ptr<OI> OI) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterHood{ShooterHood},
  m_OI{OI} {
  frc::SmartDashboard::PutNumber("Hood Position", 0);
  AddRequirements({m_shooterHood.get()});
  }


// Called when the command is initially scheduled.
void HoodTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HoodTeleop::Execute() {
  
  XButton = m_OI->GetDriverXButton();

  if (XButton) {
    m_shooterHood.get()->SetTargetPosition(units::angle::radian_t{frc::SmartDashboard::GetNumber("Hood Position", 0)});
  }
  else {
    m_shooterHood.get()->SetTargetPosition(units::angle::radian_t{0});
  }
}

// Called once the command ends or is interrupted.
void HoodTeleop::End(bool interrupted) {
  m_shooterHood.get()->SetTargetPosition(units::angle::radian_t{0});
}

// Returns true when the command should end.
bool HoodTeleop::IsFinished() {
  
  if (!XButton) {
    return true;
  }

  return false;
}
