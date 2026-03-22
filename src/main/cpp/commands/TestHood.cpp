// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TestHood.h"
#include <algorithm>

TestHood::TestHood(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<OI>&  OI) :
  m_shooterHood(shooterHood),
  m_OI(OI),
  level(0) {
    lastLeftBumper = false;
    lastRightBumper = false;
    AddRequirements({m_shooterHood.get()});
  }


// Called when the command is initially scheduled.
void TestHood::Initialize() {
  level = 0;
}

// Called repeatedly when this Command is scheduled to run
void TestHood::Execute() {
  
  auto LeftBumper = m_OI->GetOperatorLeftBumper();
  auto RightBumper = m_OI->GetOperatorRightBumper();

  if (LeftBumper && !lastLeftBumper) {
    ++level;
  }

  if (RightBumper && !lastRightBumper) {
    --level;
  }

  level = std::clamp(level, 0, MaxLevel);

  m_shooterHood->SetCommand(ShooterHood::maxPosition - level * ScaleFactor);
  frc::SmartDashboard::PutNumber("TestHood/level", level);
  frc::SmartDashboard::PutNumber("TestHood/position", (ShooterHood::maxPosition - level * ScaleFactor).value());

  lastLeftBumper = LeftBumper;
  lastRightBumper = RightBumper;
}

// Called once the command ends or is interrupted.
void TestHood::End(bool interrupted) {
  m_shooterHood.get()->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool TestHood::IsFinished() {
  return false;
}
