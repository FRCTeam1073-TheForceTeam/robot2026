// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeTeleop.h"

IntakeTeleop::IntakeTeleop(std::shared_ptr<Intake> Intake) :
    m_intake{Intake} {
    AddRequirements({m_intake.get()});
}

// Called when the command is initially scheduled.
void IntakeTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeTeleop::Execute() {
  m_intake->SetIntakeVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void IntakeTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeTeleop::IsFinished() {
  return false;
}
