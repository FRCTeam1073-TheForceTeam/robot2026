// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeTeleop.h"

IntakeTeleop::IntakeTeleop(std::shared_ptr<Intake>& intake, std::shared_ptr<OI>& oi) :
    m_intake(intake),
    m_oi(oi),
    position_in(true),
    last_button_A(false) {

    AddRequirements({m_intake.get()});
}

// Called when the command is initially scheduled.
void IntakeTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeTeleop::Execute() {

  bool button_A = m_oi->GetDriverAButton();

  if (!last_button_A && button_A) {
    // Toggle position:
    position_in = !position_in;
  }
  last_button_A = button_A; // Keep track of button for toggle.

  
  if (position_in) {  
    m_intake->SetCommand(0.0_rad);
  } else {
    m_intake->SetCommand(1.5_rad);
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
