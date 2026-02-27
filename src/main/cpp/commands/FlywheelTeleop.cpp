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
  maxVel = 10_mps;
  scale = 1.0;
  LastDPadUpState = false;
  LastDPadDownState = false;
  AddRequirements({m_flywheel.get()});
}

// Called wh(std::shared_ptr<Drivetrain> drivetrainen the command is initially scheduled.
void FlywheelTeleop::Initialize() {
  m_flywheel->SetCommand(0_mps);
}

// Called repeatedly when this Command is scheduled to run
void FlywheelTeleop::Execute() {
  XButton = m_OI->GetOperatorXButton();
  DPadUp = m_OI->GetOperatorDPadUp();
  DPadDown = m_OI->GetOperatorDPadDown();

  if (XButton) {
    frc::SmartDashboard::PutNumber("FlywheelTeleop/CommandedVelocity", maxVel.value());
    m_flywheel->SetCommand(scale * maxVel);
  }
  else{
    frc::SmartDashboard::PutNumber("FlywheelTeleop/CommandedVelocity", 0.0);
    m_flywheel->SetCommand(0_mps);
  }

  if (m_OI->GetOperatorDPadUp() && !LastDPadUpState && scale <= 0.9) {
    scale += 0.1;
    LastDPadUpState = true;
  }
  else if (m_OI->GetOperatorDPadDown() && !LastDPadDownState && scale >= 0.1) {
    scale -= 0.1;
    LastDPadDownState = true;
  }
  else {
    if (!DPadUp) {
      LastDPadUpState = false;
    }
    if (!DPadDown) {
      LastDPadDownState = false;
    }
  }

  frc::SmartDashboard::PutNumber("Flywheel/Speed Level", scale);
}

// Called once the command ends or is interrupted.
void FlywheelTeleop::End(bool interrupted) {
  m_flywheel->SetCommand(std::monostate()); // Coast/no-command.
}

// Returns true when the command should end.
bool FlywheelTeleop::IsFinished() {
  return false; //TODO: return true when finished
}
