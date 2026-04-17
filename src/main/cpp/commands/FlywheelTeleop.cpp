// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "commands/FlywheelTeleop.h"
#include <frc/DriverStation.h>
#include <iostream>
#include "subsystems/BallisticShot.h"

FlywheelTeleop::FlywheelTeleop(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<OI>& oi, std::shared_ptr<TargetFinder>& tf, std::shared_ptr<ShooterTable>& st, std::shared_ptr<BallisticShot>& bs) :
  m_flywheel(flywheel),
  m_OI(oi),
  m_tf(tf),
  m_bs(bs),
  m_st(st) {
  maxVel = 10_mps;
  scale = 1.0;
  level = 0;
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
  
  if (std::abs(m_OI->GetOperatorLeftTrigger()) >= 0.1) {
    auto feedback = m_tf->getFeedback();

    if (feedback.passing) {
      if ((feedback.rangeToTarget) < 270_in) {
        m_flywheel->SetCommand(9.0_mps);
      } else {
        m_flywheel->SetCommand(15.0_mps);
      }

    } else if (m_OI->BallisticShotMode()) {
      // Use ballistic shot:
      auto shot = m_bs->GetShot();
      m_flywheel->SetCommand(shot.FlywheelSpeed);
    } else {
      // Using lookup table:
      auto speed = m_st->GetFlywheelVelocity(feedback.rangeToTarget);
      m_flywheel->SetCommand(speed);
    }
  } else if (m_OI->GetOperatorYButton()) {
    auto speed = 9.8_mps; // Corner Shot
    m_flywheel->SetCommand(speed);
  } else if(m_OI->GetOperatorXButton()) {
    auto speed = 9.2_mps; // Tower Shot
    m_flywheel->SetCommand(speed);
  } else {
    m_flywheel->SetCommand(std::monostate());
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
