// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroClimber.h"
#include <units/math.h>
#include <iostream>

ZeroClimber::ZeroClimber(std::shared_ptr<Climber> climber, bool unsafe) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_climber(climber) {
    if (!unsafe) {
      AddRequirements({m_climber.get()});
    }
}

// Called when the command is initially scheduled.
void ZeroClimber::Initialize() {
  std::cerr << "Zero Climber" << std::endl;

}

// Called repeatedly when this Command is scheduled to run
void ZeroClimber::Execute() {
  auto velocity = -0.03_mps;
  m_climber->SetCommand(velocity);

}

// Called once the command ends or is interrupted.
void ZeroClimber::End(bool interrupted) {
  if (interrupted) {
    std::cerr << "Zero Climber Interrupted" << std::endl;
  } else {
    std::cerr << "Zero Climber Finished" << std::endl;
  }
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
