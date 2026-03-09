// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SmartDashPrint.h"

SmartDashPrint::SmartDashPrint(const std::string& s) :
  s(s)
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SmartDashPrint::Initialize() {
  frc::SmartDashboard::PutString("Auto Event", s);
}

// Called repeatedly when this Command is scheduled to run
void SmartDashPrint::Execute() {}

// Called once the command ends or is interrupted.
void SmartDashPrint::End(bool interrupted) {}

// Returns true when the command should end.
bool SmartDashPrint::IsFinished() {
  return true;
}
