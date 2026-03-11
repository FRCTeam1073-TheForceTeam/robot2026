// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TestFlywheel.h"
#include <frc/DriverStation.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


TestFlywheel::TestFlywheel(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<OI>& oi) :
  m_flywheel(flywheel),
  m_OI(oi),
  level(0) {
  lastDPadUp = false;
  lastDPadDown = false;
  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void TestFlywheel::Initialize() {
  level = 0;
}

// Called repeatedly when this Command is scheduled to run
void TestFlywheel::Execute() {
  auto DPadUp = m_OI->GetOperatorDPadUp();
  auto DPadDown = m_OI->GetOperatorDPadDown();

  if (DPadUp && !lastDPadUp) {
    ++level;
  }
  
  if (DPadDown && !lastDPadDown) {
    --level;
  }

  level = std::clamp(level, 0, MaxLevel);

  m_flywheel->SetCommand(level * ScaleFactor);
  frc::SmartDashboard::PutNumber("TestFlywheel/level", level);
  frc::SmartDashboard::PutNumber("TestFlywheel/speed", level * ScaleFactor.value());

  // Change detector:
  lastDPadDown = DPadDown;
  lastDPadUp = DPadUp;
 }

// Called once the command ends or is interrupted.
void TestFlywheel::End(bool interrupted) {
  m_flywheel->SetCommand(std::monostate()); // Coast/no-command.
}

// Returns true when the command should end.
bool TestFlywheel::IsFinished() {
  return false; //TODO: return true when finished
}
