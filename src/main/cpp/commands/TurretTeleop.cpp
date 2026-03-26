// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurretTeleop.h"
#include <iostream>
#include <frc/DriverStation.h>

//TODO: finish the command; it is not complete yet

TurretTeleop::TurretTeleop(std::shared_ptr<Turret>& turret, std::shared_ptr<OI>& oi, std::shared_ptr<TargetFinder>& targetFinder) :
  m_turret(turret),
  m_OI(oi),
  m_targetFinder(targetFinder) {
  lastError = 0,
  targetAngle = 0_rad,
  AddRequirements(m_turret.get());
}

void TurretTeleop::Initialize() {
    std::cerr << "TeleopTurret: Init" << std::endl;
  }

// Called repeatedly when this Command is scheduled to run
void TurretTeleop::Execute() {

  leftX = m_OI->GetOperatorLeftX();
  

  if (std::abs(leftX) > 0.1) {
    targetAngle = 1.5_rad * leftX;
  }
  else if(m_OI->GetOperatorLeftTrigger() >= 0.1) {
    targetAngle = units::radian_t(m_targetFinder->getFeedback().turretToTargetAngle.value());
  }
  else {
    targetAngle = 0_rad;
  }
  
  m_turret->SetCommand(targetAngle);

  frc::SmartDashboard::PutNumber("TeleopTurret/targetAngle", targetAngle.value());
}

// Called once the command ends or is interrupted.
void TurretTeleop::End(bool interrupted) {
  if (interrupted) {
        std::cerr << "TeleopTurret: Interrupted!" << std::endl;
    }
    m_turret->SetCommand(std::monostate());
  }

// Returns true when the command should end.
bool TurretTeleop::IsFinished() {
  return false;//TODO: return true if it finishes
}
