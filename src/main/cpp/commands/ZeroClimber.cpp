// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroClimber.h"
#include <units/math.h>

ZeroClimber::ZeroClimber(std::shared_ptr<Climber> climber) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_climber(climber) {
  AddRequirements({m_climber.get()});
}

// Called when the command is initially scheduled.
void ZeroClimber::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ZeroClimber::Execute() {
  auto velocity = -0.03_mps;
  m_climber->SetCommand(velocity);
}

// Called once the command ends or is interrupted.
void ZeroClimber::End(bool interrupted) {
  m_climber->Zero();
  m_climber->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool ZeroClimber::IsFinished() {
  if(units::math::abs(m_climber->GetFeedback().force) > 6_N) { 
    return true;
  }
  return false; 
}
