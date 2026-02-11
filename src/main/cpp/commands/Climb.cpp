// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"

Climb::Climb(std::shared_ptr<Climber> climber) :
  m_climber{climber} {
  AddRequirements({m_climber.get()});
   //Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  m_climber->SetVelocity(1.0_tps); //TODO: change this to a number that makes sense
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  if (m_climber->GetVoltage() > units::volt_t(5)) { //TODO: Get Value

    m_climber->SetVoltage(0.0_V);
    m_climber->StopMotor();

  }

  if (!m_climber->IsHooked()) {
    
    m_climber->SetVelocity(-1.0_tps);

    if (m_climber->GetVoltage() > units::volt_t(5)) { //TODO: Get Value

      m_climber->SetVoltage(0.0_V);
      m_climber->StopMotor();

    }
  }
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return false;
}
