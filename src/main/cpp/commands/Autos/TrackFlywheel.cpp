// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/TrackFlywheel.h"
#include "utilities/BallisticShot.h"


TrackFlywheel::TrackFlywheel(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st, bool lookupTable) :
 m_flywheel(flywheel),
 m_hf(hf),
 m_st(st),
 m_lookupTable(lookupTable)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_flywheel.get());
}

// Called when the command is initially scheduled.
void TrackFlywheel::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TrackFlywheel::Execute() {
  units::length::meter_t range = m_hf -> getFeedback().rangeToTarget;

  if (m_lookupTable)
  {
    units::velocity::meters_per_second_t targetSpeed = m_st -> GetFlywheelVelocity(range);
      m_flywheel -> SetCommand(targetSpeed);
  }
  else
  {
    auto shot = BallisticShot::GetShot(range); 
    m_flywheel -> SetCommand(shot.FlywheelSpeed);
  }

}

// Called once the command ends or is interrupted.
void TrackFlywheel::End(bool interrupted) {

  m_flywheel -> SetCommand(std::monostate());
}

// Returns true when the command should end.
bool TrackFlywheel::IsFinished() {
  return false;
}
