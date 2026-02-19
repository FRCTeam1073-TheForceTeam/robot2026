// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurretTeleop.h"
#include <iostream>
#include <frc/DriverStation.h>

//TODO: finish the command; it is not complete yet

TurretTeleop::TurretTeleop(std::shared_ptr<Turret>& turret, std::shared_ptr<OI>& oi) :
  m_turret(turret),
  m_OI(oi) {
  lastError = 0,
  isAlignedToHub = false,
  angularVel = 0_rad_per_s,
  targetAngle = 0_rad,
  targetPosition = 0_rad,
  position = 0_rad,//zeroed position is touching the hard stop
  minPosition = 0_rad,
  maxPosition = 6_rad,//TODO: get maximum velocity form EM
  AddRequirements({m_turret.get(), m_OI.get()});
}

void TurretTeleop::Initialize() {
    std::cerr << "RotateTurret Init" << std::endl;
  }

// Called repeatedly when this Command is scheduled to run
void TurretTeleop::Execute() {
  //TODO: determine direction that robot must be facing in & use that to automatically set the angle
  leftX = m_OI->GetOperatorLeftX();

  if (leftX < -0.1 && position > minPosition){
    targetAngle = leftX * 1_rad;//TODO: Change the scale factor that the x value is multiplied by so that the turret moves at a reasonable speed 
  }
  else if (leftX > 0.1 && position < maxPosition){
    targetAngle = leftX * 1_rad;//TODO: Change the scale factor that the x value is multiplied by so that the turret moves at a reasonable speed 
  }
  else{
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
  if(interrupted) {
        std::cerr << "RotateTurret: Interrupted!" << std::endl;
    }
    Command::End(interrupted);
  }

// Returns true when the command should end.
bool TurretTeleop::IsFinished() {
  return false;//TODO: return true if it finishes
}
