// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Laser.h"

Laser::Laser(std::shared_ptr<LaserCan> laserCan) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_laser.get()});
}

// Called when the command is initially scheduled.
void Laser::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Laser::Execute(
  
) {}

// Called once the command ends or is interrupted.
void Laser::End(bool interrupted) {}

// Returns true when the command should end.
bool Laser::IsFinished() {
  return false;
}
