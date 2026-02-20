// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroClimber.h"

ZeroClimber::ZeroClimber(std::shared_ptr<Climber> climber) :
  m_climber{climber}
{
    AddRequirements({m_climber.get()});
}

// Called when the command is initially scheduled.
void ZeroClimber::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ZeroClimber::Execute() {
    m_climber->SetCommand(-1.0_mps);
}

// Called once the command ends or is interrupted.
void ZeroClimber::End(bool interrupted) {}

// Returns true when the command should end.
bool ZeroClimber::IsFinished() {
    if (m_climber->GetVoltage() > 5_V) { // TODO: Test this value
        m_climber->SetCommand(0.0_mps);
        m_climber->ZeroPosition();
        return true;
    } else {
        return false;
    }
}
