// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraight.h"

DriveStraight::DriveStraight(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer) :
  m_drivetrain(drivetrain),
  m_localizer(localizer)
{
  // Use addRequirements() here to declare subsystem dependencies.
  startTime = 0_s;
  AddRequirements({m_drivetrain.get(), m_localizer.get()});
}

// Called when the command is initially scheduled.
void DriveStraight::Initialize() {
  startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void DriveStraight::Execute() {
  m_drivetrain->SetChassisSpeeds(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, m_localizer->getPose().Rotation())
  );
}

// Called once the command ends or is interrupted.
void DriveStraight::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveStraight::IsFinished() {
  if(frc::Timer::GetFPGATimestamp() - startTime >= 5_s ) {
    return true;
  }
  return false;
}
