// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HoodTeleop.h"

HoodTeleop::HoodTeleop(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<OI>&  OI) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterHood(shooterHood),
  m_OI(OI) {
    level = 0;
    RightBumperPastState = false;
    LeftBumperPastState = false;
    AddRequirements({m_shooterHood.get()});
  }


// Called when the command is initially scheduled.
void HoodTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HoodTeleop::Execute() {
  
  LeftBumper = m_OI->GetOperatorLeftBumper();
  RightBumper = m_OI->GetOperatorRightBumper();

  if (RightBumper && level < maxLevel && !RightBumperPastState) {
    level += 1;
    RightBumperPastState = true;
  }
  else if (LeftBumper && level > 0 && !LeftBumperPastState){
    level -= 1;
    LeftBumperPastState = true;
  } else {
    if (!RightBumper){
      RightBumperPastState = false;
    } 
    if (!LeftBumper){
      LeftBumperPastState = false;
    }
  }
  

  m_shooterHood->SetCommand(level * ScaleFactor);
  frc::SmartDashboard::PutNumber("HoodTeleop/hood level", level);
  frc::SmartDashboard::PutNumber("HoodTeleop/hood position", level * ScaleFactor.value());
}

// Called once the command ends or is interrupted.
void HoodTeleop::End(bool interrupted) {
  m_shooterHood.get()->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool HoodTeleop::IsFinished() {
  
  if (!LeftBumper && !RightBumper && (m_shooterHood->GetFeedback().position - level * ScaleFactor).value() <= 0.08 && (m_shooterHood->GetFeedback().position - level * ScaleFactor).value() >= -0.08) { //TODO: change how precise the it is  
    return true;
  }

  return false;
}
