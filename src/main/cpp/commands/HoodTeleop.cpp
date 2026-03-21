// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoodTeleop.h"

HoodTeleop::HoodTeleop(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<OI>&  OI, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<BallisticShot>& bs, std::shared_ptr<ZoneFinder>& zone) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterHood(shooterHood),
  m_OI(OI),
  m_hf(hf),
  m_bs(bs),
  m_zone(zone) {
    level = 0;
    RightBumperPastState = false;
    LeftBumperPastState = false;
    AddRequirements({m_shooterHood.get()});
  }


// Called when the command is initially scheduled.
void HoodTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HoodTeleop::Execute() {
  
  
  if(m_zone->GetZones().contains("TRENCH"))
  {
    level = 0;
  }
  else if (std::abs(m_OI->GetOperatorLeftTrigger()) >= 0.1) {
    // Use lookup table:
    auto range = m_hf->getFeedback().rangeToTarget;
    auto shot = m_bs->GetShot(range, 0.5_m);//TODO: find a good value for the height above the hub
    auto angle = shot.HoodAngle;
    m_shooterHood->SetCommand(angle);
  } else if (m_OI->GetOperatorYButton()) {
    auto angle = 0.2188_rad; // Corner Shot
    m_shooterHood->SetCommand(angle);
  } else if (m_OI->GetOperatorXButton()) {
    auto angle = 0.1408_rad; // Tower Shot
    m_shooterHood->SetCommand(angle);
  } else {
    m_shooterHood->SetCommand(0.0_rad);
  }

  // Old Code
  // } else {
  //   LeftBumper = m_OI->GetOperatorLeftBumper();
  //   RightBumper = m_OI->GetOperatorRightBumper();

  //   if (RightBumper && level < maxLevel && !RightBumperPastState) {
  //     level += 1;
  //     RightBumperPastState = true;
  //   }
  //   else if (LeftBumper && level > 0 && !LeftBumperPastState){
  //     level -= 1;
  //     LeftBumperPastState = true;
  //   } else {
  //     if (!RightBumper){
  //       RightBumperPastState = false;
  //     } 
  //     if (!LeftBumper){
  //       LeftBumperPastState = false;
  //     }
  //   }
    
  //   m_shooterHood->SetCommand(level * ScaleFactor);
  //   frc::SmartDashboard::PutNumber("HoodTeleop/hood level", level);
  //     frc::SmartDashboard::PutNumber("HoodTeleop/hood position", level * ScaleFactor.value());
   
}

// Called once the command ends or is interrupted.
void HoodTeleop::End(bool interrupted) {
  m_shooterHood.get()->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool HoodTeleop::IsFinished() {
  return false;
}
