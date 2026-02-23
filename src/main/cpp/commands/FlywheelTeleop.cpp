// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FlywheelTeleop.h"
#include <frc/DriverStation.h>
#include <iostream>

#include <choreo/Choreo.h>
#include <commands/Autos/TestAuto.h>

FlywheelTeleop::FlywheelTeleop(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<OI>& oi) :
  m_flywheel(flywheel),
  m_OI(oi) {
  maxVel = 20_mps; //TODO: test and change this value
  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void FlywheelTeleop::Initialize() {
  m_flywheel->SetCommand(0_mps);
}

// Called repeatedly when this Command is scheduled to run
void FlywheelTeleop::Execute() {
  XButton = m_OI->GetOperatorXButton();
  if (XButton) {
    frc::SmartDashboard::PutNumber("FlywheelTeleop/CommandedVelocity", maxVel.value());
    m_flywheel->SetCommand(maxVel);
  }
  else{
    frc::SmartDashboard::PutNumber("FlywheelTeleop/CommandedVelocity", 0.0);
    m_flywheel->SetCommand(0_mps);
  }
}

// Called once the command ends or is interrupted.
void FlywheelTeleop::End(bool interrupted) {
  m_flywheel->SetCommand(std::monostate()); // Coast/no-command.
}

// Returns true when the command should end.
bool FlywheelTeleop::IsFinished() {
  return false; //TODO: return true when finished
}
