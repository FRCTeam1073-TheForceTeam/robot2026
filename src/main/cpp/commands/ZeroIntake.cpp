// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroIntake.h"
#include <units/math.h>

ZeroIntake::ZeroIntake(std::shared_ptr<Intake> intake) :
  m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_intake.get()});
}

// Called when the command is initially scheduled.
void ZeroIntake::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ZeroIntake::Execute() {
  auto velocity = -2_rad_per_s;
  m_intake->SetCommand(velocity);
}

// Called once the command ends or is interrupted.
void ZeroIntake::End(bool interrupted) {
  m_intake->Zero();
  m_intake->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool ZeroIntake::IsFinished() {
  if(units::math::abs(m_intake->GetFeedback().torque) > 2.5_Nm) {
    return true;
  }
  return false;
}
