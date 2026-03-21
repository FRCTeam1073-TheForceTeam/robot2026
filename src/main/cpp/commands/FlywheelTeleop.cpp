// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FlywheelTeleop.h"
#include <frc/DriverStation.h>
#include <iostream>

#include <choreo/Choreo.h>
#include <commands/Autos/TestAuto.h>

FlywheelTeleop::FlywheelTeleop(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<OI>& oi, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<BallisticShot>& bs) :
  m_flywheel(flywheel),
  m_OI(oi),
  m_hf(hf),
  m_bs(bs) {
  maxVel = 10_mps;
  scale = 1.0;
  level = 0;
  LastDPadUpState = false;
  LastDPadDownState = false;
  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void FlywheelTeleop::Initialize() {
  m_flywheel->SetCommand(0_mps);
}

// Called repeatedly when this Command is scheduled to run
void FlywheelTeleop::Execute() {
  
  if (std::abs(m_OI->GetOperatorLeftTrigger()) >= 0.1) {
    // Using lookup table:
    auto range = m_hf->getFeedback().rangeToTarget;
    auto shot = m_bs->GetShot(range, 0.5_m); //TODO: find a good value for the height above the hub
    auto speed = shot.FlywheelSpeed;
    m_flywheel->SetCommand(speed);
  } else if (m_OI->GetOperatorYButton()) {
    auto speed = 11.0_mps; // Corner Shot
    m_flywheel->SetCommand(speed);
  } else if(m_OI->GetOperatorXButton()) {
    auto speed = 9.4_mps; // Tower Shot
    m_flywheel->SetCommand(speed);
  } else {
    m_flywheel->SetCommand(std::monostate());
  }

/*
   if (m_OI->GetOperatorLeftTrigger()>= 0.1) {
  //   // Use lookup table:
     auto range = m_hf->getFeedback().rangeToTarget;
     auto speed = m_st->GetFlywheelVelocity(range);
     m_flywheel->SetCommand(speed);
   } else if(m_OI->GetOperatorYButton()){
      auto speed = 11_mps; //Corner shot
      m_flywheel->SetCommand(speed);
   }else if(m_OI->GetOperatorXButton()){
      auto speed = 9.4_mps; //Tower shot
      m_flywheel->SetCommand(speed);
//    m_flywheel->SetCommand(level * scaleFactor);
//    frc::SmartDashboard::PutNumber("Flywheel/Speed Level", level);
//    frc::SmartDashboard::PutNumber("Flywheel/Speed", level * scaleFactor.value());
*/
}

// Called once the command ends or is interrupted.
void FlywheelTeleop::End(bool interrupted) {
  m_flywheel->SetCommand(std::monostate()); // Coast/no-command.
}

// Returns true when the command should end.
bool FlywheelTeleop::IsFinished() {
  return false; //TODO: return true when finished
}
