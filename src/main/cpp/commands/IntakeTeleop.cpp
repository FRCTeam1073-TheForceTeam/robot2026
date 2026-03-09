// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeTeleop.h"

IntakeTeleop::IntakeTeleop(std::shared_ptr<Intake>& intake, std::shared_ptr<OI>& oi, std::shared_ptr<ZoneFinder>& zone) :
    m_intake(intake),
    m_oi(oi),
    m_zone(zone),
    position_in(true),
    last_bumper_right(false) {

    AddRequirements({m_intake.get()});
}

// Called when the command is initially scheduled.
void IntakeTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeTeleop::Execute() {

  bool bumper_right = m_oi->GetDriverRightBumper();

  if (m_zone->GetZones().contains("TRENCH"))
  {
    position_in = false;
  }
  else if (!bumper_right && bumper_right) {
    // Toggle position:
    position_in = !position_in;
  }
  last_bumper_right = bumper_right; // Keep track of button for toggle.

  
  if (position_in) {  
    m_intake->SetCommand(-122.0_deg);
  } else {
    m_intake->SetCommand(-0.1_deg);
  }
}

// Called once the command ends or is interrupted.
void IntakeTeleop::End(bool interrupted) {
  m_intake->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool IntakeTeleop::IsFinished() {
  return false;
}
