// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurretTeleop.h"
#include <iostream>
#include <frc/DriverStation.h>

//TODO: finish the command; it is not complete yet

TurretTeleop::TurretTeleop(std::shared_ptr<Turret>& turret, std::shared_ptr<OI>& oi, std::shared_ptr<HubFinder>& hubFinder) :
  m_turret(turret),
  m_OI(oi),
  m_hubFinder(hubFinder) {
  lastError = 0,
  isAlignedToHub = false,
  angularVel = 0_rad_per_s,
  targetAngle = 0_rad,
  targetPosition = 0_rad,
  position = 0_rad,//zeroed position is touching the hard stop
  minPosition = 0_rad,
  maxPosition = 6_rad,//TODO: get maximum velocity form EM
  AddRequirements(m_turret.get());
}

void TurretTeleop::Initialize() {
    std::cerr << "RotateTurret Init" << std::endl;
  }

// Called repeatedly when this Command is scheduled to run
void TurretTeleop::Execute() {
  //TODO: determine direction that robot must be facing in & use that to automatically set the angle
  leftX = m_OI->GetOperatorLeftX();

  if (std::abs(leftX) > 0.1) {
    targetAngle = 1.5 * leftX * 1_rad;
  }
  else if(m_OI->GetOperatorDPadLeft()) {
    targetAngle = units::radian_t(std::clamp(m_hubFinder->getTurretToHubAngle().value(), -2 * std::numbers::pi / 3, std::numbers::pi / 2));
  }
  else {
    targetAngle = 0_rad;
  }
  
  m_turret->SetCommand(targetAngle);

  frc::SmartDashboard::PutNumber("Turret/angularVel", angularVel.value());
  frc::SmartDashboard::PutNumber("Turret/targetAngle", targetAngle.value());
  frc::SmartDashboard::PutNumber("Turret/position", position.value());
  frc::SmartDashboard::PutNumber("Turret/targetPosition", targetPosition.value());
  frc::SmartDashboard::PutNumber("Turret/leftX", leftX);
  frc::SmartDashboard::PutBoolean("TeleopDrive/isAlignedToHub", isAlignedToHub);
  //m_turret->SetTargetAngle(targetPosition); // use this line of code once the localizer is added

}

// Called once the command ends or is interrupted.
void TurretTeleop::End(bool interrupted) {
  if (interrupted) {
        std::cerr << "RotateTurret: Interrupted!" << std::endl;
    }
    m_turret->SetCommand(std::monostate());
  }

// Returns true when the command should end.
bool TurretTeleop::IsFinished() {
  return false;//TODO: return true if it finishes
}
