// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeTeleop.h"
#include <iostream>
#include <frc/DriverStation.h>

IntakeTeleop::IntakeTeleop(std::shared_ptr<Intake> Intake, std::shared_ptr<OI> OI) :
  m_intake{Intake}, 
  m_OI{OI} {
  WheelAngularVel = 0_rad_per_s,
  WheelTargetAngularVel = 0_rad_per_s,
  ArmTargetPosition = 0_rad,
  ArmPosition = 0_rad,
  ArmMinPosition = 0_rad,//TODO: get minimum position from EM
  ArmMaxPosition = 2_rad,//TODO: get maximum position from EM
  AddRequirements({m_intake.get(), m_OI.get()});
}

// Called when the command is initially scheduled.
void IntakeTeleop::Initialize() {
  std::cerr << "IntakeTeleop Init" << std::endl;
    Command::Initialize();
  }

// Called repeatedly when this Command is scheduled to run
void IntakeTeleop::Execute() {
  if(!AButton==m_OI->GetDriverAButton())
    down=!down;
  AButton = m_OI->GetDriverBButton();
  if(!BButton==m_OI->GetDriverBButton())
    spinning=!spinning;
  BButton = m_OI->GetDriverBButton();
  WheelAngularVel = m_intake->GetIntakeVelocityTurns();
  ArmPosition = m_intake->GetArmPosition();
  if(down)
    m_intake->SetTargetArmPosition(ArmMaxPosition);
  else
    m_intake->SetTargetArmPosition(ArmMinPosition);
  if(spinning)
    m_intake->SetIntakeVelocity(8.0_rad_per_s);
  else
    m_intake->SetIntakeVelocity(0.0_rad_per_s);
}

// Called once the command ends or is interrupted.
void IntakeTeleop::End(bool interrupted) {
  if(interrupted) {
        std::cerr << "IntakeTeleop: Interrupted!" << std::endl;
    }
    Command::End(interrupted);
  }


// Returns true when the command should end.
bool IntakeTeleop::IsFinished() {
  return false;//TODO: return true if it finishes
}
