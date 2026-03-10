// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberTeleop.h"

ClimberTeleop::ClimberTeleop(std::shared_ptr<Climber>& climber, std::shared_ptr<OI>& oi, std::shared_ptr<ZoneFinder>& zone) :
  m_climber(climber),
  m_OI(oi),
  m_zone(zone)
   {

  AddRequirements({m_climber.get()});
}

// Called when the command is initially scheduled.
void ClimberTeleop::Initialize() {
  currentPosition = m_climber->getClimberPosition();
}

// Called repeatedly when this Command is scheduled to run
void ClimberTeleop::Execute() {

  // rightY = m_OI->GetOperatorRightY();

  // if(abs(rightY) < 0.1) rightY = 0.0;

  // vy = rightY*-0.67_mps;
  // m_climber->SetCommand(vy);

  currentPosition = m_climber->getClimberPosition();

  input = 0.0;
  if(m_OI->GetDriverMenuButton()) {
    input = 1.0;
  } else if (m_OI->GetDriverViewButton()) {
    input = -1.0;
  } else {
    input = 0.0;
  }
  
  commandedPosition = currentPosition + (input * 0.1_m);
  
  if(m_zone->GetZones().contains("TRENCH"))
  {
    commandedPosition = 0_m;
  }
  m_climber->SetCommand(commandedPosition);





  //9 amps at bottom
  //10 when lift (but have switch)
}

// Called once the command ends or is interrupted.
void ClimberTeleop::End(bool interrupted) {
  m_climber->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool ClimberTeleop::IsFinished() {
  return false;
}
