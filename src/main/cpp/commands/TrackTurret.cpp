// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TrackTurret.h"
#include <iostream>
#include <frc/DriverStation.h>


TrackTurret::TrackTurret(std::shared_ptr<Turret>& turret, std::shared_ptr<HubFinder>& hubFinder) :
  m_turret(turret),
  m_hubFinder(hubFinder) 
  {
  lastError = 0,
  isAlignedToHub = false,
  targetPosition = 0_rad,
  position = 0_rad,//zeroed position is touching the hard stop
  minPosition = 0_rad,
  maxPosition = 6_rad,//TODO: get maximum velocity form EM
  AddRequirements(m_turret.get());
}

// Called when the command is initially scheduled.
void TrackTurret::Initialize() {
  std::cerr << "TrackTurret Init" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void TrackTurret::Execute() {
  targetPosition = m_hubFinder->getFeedback().turretToHubAngle;
  if (targetPosition >= minPosition && targetPosition <= maxPosition) {
    m_turret->SetCommand(targetPosition);
  }

  frc::SmartDashboard::PutNumber("Turret/position", position.value());
  frc::SmartDashboard::PutNumber("Turret/targetPosition", targetPosition.value());
}

// Called once the command ends or is interrupted.
void TrackTurret::End(bool interrupted) {
  if (interrupted) {
        std::cerr << "TrackTurret: Interrupted!" << std::endl;
    }
    m_turret->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool TrackTurret::IsFinished() {
  return false;
}
