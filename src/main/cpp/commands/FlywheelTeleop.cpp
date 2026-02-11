// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FlywheelTeleop.h"

FlywheelTeleop::FlywheelTeleop(std::shared_ptr<Flywheel> flywheel) :
  m_flywheel{flywheel} {

  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void FlywheelTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FlywheelTeleop::Execute() {
  m_flywheel->SetVelocity(0.0_tps);
}

// Called once the command ends or is interrupted.
void FlywheelTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool FlywheelTeleop::IsFinished() {
  return false;
}
