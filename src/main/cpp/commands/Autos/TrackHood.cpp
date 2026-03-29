// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/Autos/TrackHood.h"
#include "utilities/BallisticShot.h"


TrackHood::TrackHood(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st, bool shooterTable) :
m_shooterHood(shooterHood),
m_hf(hf),
m_st(st),
m_shooterTable(shooterTable) {
  AddRequirements(m_shooterHood.get());
}

// Called when the command is initially scheduled.
void TrackHood::Initialize() {}


// Called repeatedly when this Command is scheduled to run
void TrackHood::Execute() {
  units::length::meter_t range = m_hf->getFeedback().rangeToTarget;
  if(m_shooterTable)
  {
    units::angle::radian_t targetAngle = m_st->GetHoodAngle(range);
    m_shooterHood->SetCommand(targetAngle);
  }
  else
  {
    auto shot = BallisticShot::GetShot(range);
    m_shooterHood->SetCommand(shot.HoodAngle);
  }
}

// Called once the command ends or is interrupted.
void TrackHood::End(bool interrupted) {
  m_shooterHood->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool TrackHood::IsFinished() {
  return false;
}