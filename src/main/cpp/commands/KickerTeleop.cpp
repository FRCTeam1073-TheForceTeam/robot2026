// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/KickerTeleop.h"

KickerTeleop::KickerTeleop(std::shared_ptr<Kicker>& kicker, std::shared_ptr<OI>& oi) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_kicker(kicker),
  m_OI(oi) {
  AddRequirements({m_kicker.get()});
}
// Called when the command is initially scheduled.
void KickerTeleop::Initialize() {
  targetVelocity = 0_mps;
}

// Called repeatedly when this Command is scheduled to run
void KickerTeleop::Execute() {
  auto BButton = m_OI->GetOperatorBButton();

  if (m_OI->GetOperatorRightTrigger() >= 0.1) {
    // targetVelocity = 5.85_mps;
    // targetVelocity = 7.3_mps; //was 5.85
    targetVelocity = 7.0_mps;

    m_kicker->SetCommand(targetVelocity);
  } else if (BButton) {
    targetVelocity = -1.65_mps;
    m_kicker->SetCommand(targetVelocity);
  } else {
    targetVelocity = 0.0_mps;
    m_kicker->SetCommand(std::monostate());
  }
}

// Called once the command ends or is interrupted.
void KickerTeleop::End(bool interrupted) {
  m_kicker->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool KickerTeleop::IsFinished() {
  return false;
}
