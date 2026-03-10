// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoodTeleop.h"

HoodTeleop::HoodTeleop(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<OI>&  OI, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st, std::shared_ptr<ZoneFinder>& zone) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterHood(shooterHood),
  m_OI(OI),
  m_hf(hf),
  m_st(st),
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
  
   if (m_OI->GetOperatorDPadLeft()) {
    // Use lookup table:
    auto range = m_hf->getFeedback().rangeToHub;
    auto angle = m_st->GetHoodAngle(range);
    m_shooterHood->SetCommand(angle);
  } else {
    LeftBumper = m_OI->GetOperatorLeftBumper();
    RightBumper = m_OI->GetOperatorRightBumper();

    if (RightBumper && level < maxLevel && !RightBumperPastState) {
      level += 1;
      RightBumperPastState = true;
    }
    else if (LeftBumper && level > 0 && !LeftBumperPastState){
      level -= 1;
      LeftBumperPastState = true;
    } else {
      if (!RightBumper){
        RightBumperPastState = false;
      } 
      if (!LeftBumper){
        LeftBumperPastState = false;
      }
    }
    if(m_zone->GetZones().contains("TRENCH"))
    {
      level = 0;
    }
    
    m_shooterHood->SetCommand(level * ScaleFactor);
    frc::SmartDashboard::PutNumber("HoodTeleop/hood level", level);
    frc::SmartDashboard::PutNumber("HoodTeleop/hood position", level * ScaleFactor.value());
  }
}

// Called once the command ends or is interrupted.
void HoodTeleop::End(bool interrupted) {
  m_shooterHood.get()->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool HoodTeleop::IsFinished() {
  return false;
}
