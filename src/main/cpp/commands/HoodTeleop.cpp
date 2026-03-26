// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoodTeleop.h"
#include "utilities/BallisticShot.h"

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
  
  
  if(m_zone->GetZones().contains("TRENCH"))
  {
    // Put the hood "back" to be out of the way.
    m_shooterHood->SetCommand(ShooterHood::maxPosition);
  } else if (std::abs(m_OI->GetOperatorLeftTrigger()) >= 0.1) {
    auto range = m_hf->getFeedback().rangeToTarget;

    if (m_OI->BallisticShotMode()) {
      // Use ballistic shot:
      auto shot = BallisticShot::GetShot(range, 1.5_m); //TODO: change value
      m_shooterHood->SetCommand(shot.HoodAngle);
    } else {
      // Use lookup table:
      auto angle = m_st->GetHoodAngle(range);
      m_shooterHood->SetCommand(angle);
    }
  } else if (m_OI->GetOperatorYButton()) {
    // Corner shot:
    //auto angle = 0.2188_rad; // Corner Shot old
    units::angle::radian_t angle = 56.66_deg;
    m_shooterHood->SetCommand(angle);
  } else if (m_OI->GetOperatorXButton()) {
    // Tower shot:
    //auto angle = 0.1408_rad; // Tower Shot old
    units::angle::radian_t angle = 54.83_deg;
    m_shooterHood->SetCommand(angle);
  } else {
    // Put the hood "back" to be out of the way.
    m_shooterHood->SetCommand(ShooterHood::maxPosition);
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
