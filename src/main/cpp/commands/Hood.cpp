// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Hood.h"

Hood::Hood(std::shared_ptr<ShooterHood> ShooterHood) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_shooterHood.get()});

}

// Called when the command is initially scheduled.
void Hood::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Hood::Execute() {
  m_shooterHood->SetTargetPosition(units::angle::radian_t (0));
}

// Called once the command ends or is interrupted.
void Hood::End(bool interrupted) {
  m_shooterHood->SetTargetPosition(units::angle::radian_t (0));
}

// Returns true when the command should end.
bool Hood::IsFinished() {
  return false;
}
