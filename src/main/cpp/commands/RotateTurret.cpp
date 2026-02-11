// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTurret.h"
#include <iostream>
#include <frc/DriverStation.h>

//TODO: finish the command; it is not complete yet

RotateTurret::RotateTurret(std::shared_ptr<ShooterRotater> shooterRotater, std::shared_ptr<OI> oi) :
  m_shooterRotater {shooterRotater},
  m_OI{oi} {
  lastError = 0,
  isAlignedToHub = false,
  angularVel = 0_rad_per_s,
  targetAngularVel = 0_rad_per_s,
  targetPosition = 0_rad,
  position = 0_rad,//zeroed position is touching the hard stop
  minPosition = 0_rad,
  maxPosition = 6_rad,//TODO: get maximum velocity form EM
  AddRequirements({m_shooterRotater.get(), m_OI.get()});
}

void RotateTurret::Initialize() {
    std::cerr << "RotateTurret Init" << std::endl;
    Command::Initialize();
  }

// Called repeatedly when this Command is scheduled to run
void RotateTurret::Execute() {
  //TODO: determine direction that robot must be facing in & use that to automatically set the angle
  leftX = m_OI->GetOperatorLeftX();
  angularVel = m_shooterRotater->GetVelocity();
  position = m_shooterRotater->GetAngle();

  if (leftX < -0.1 && position > minPosition){
    targetAngularVel = leftX * 1_rad_per_s;//TODO: Change the scale factor that the x value is multiplied by so that the turret moves at a reasonable speed 
  }
  else if (leftX > 0.1 && position < maxPosition){
    targetAngularVel = leftX * 1_rad_per_s;//TODO: Change the scale factor that the x value is multiplied by so that the turret moves at a reasonable speed 
  }
  else{
    targetAngularVel = 0_rad_per_s;
  }

  m_shooterRotater->SetTargetVelocity(targetAngularVel);



  frc::SmartDashboard::PutNumber("RotateTurret/angularVel", angularVel.value());
  frc::SmartDashboard::PutNumber("RotateTurret/targetAngularVel", targetAngularVel.value());
  frc::SmartDashboard::PutNumber("RotateTurret/position", position.value());
  frc::SmartDashboard::PutNumber("RotateTurret/targetPosition", targetPosition.value());
  frc::SmartDashboard::PutNumber("RotateTurret/leftX", leftX);
  frc::SmartDashboard::PutBoolean("TeleopDrive/isAlignedToHub", isAlignedToHub);
  //m_shooterRotater->SetTargetAngle(targetPosition); // use this line of code once the localizer is added

}

// Called once the command ends or is interrupted.
void RotateTurret::End(bool interrupted) {
  if(interrupted) {
        std::cerr << "RotateTurret: Interrupted!" << std::endl;
    }
    Command::End(interrupted);
  }

// Returns true when the command should end.
bool RotateTurret::IsFinished() {
  return false;//TODO: return true if it finishes
}
