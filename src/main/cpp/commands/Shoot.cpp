// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"
// #include <frc/Timer.h>

Shoot::Shoot(std::shared_ptr<Flywheel> flywheel) :
  m_flywheel{flywheel} {
  AddRequirements({m_flywheel.get()});
}

// Called when the command is initially scheduled.
void Shoot::Initialize() {
  // frc::Timer().frc::Timer::Start();
  // while(!frc::Timer().frc::Timer::HasElapsed(units::second_t(1.0))) {
  //   return(true);
  // }
  sleep(4.1); //TODO: replace this with the amount of time needed for motor to start up
  
}
// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() {  
  m_flywheel->SetVelocity(6.7_tps);//TODO: change number into one that catually makes sense
  }

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) {
  m_flywheel->SetVelocity(0.0_tps);
}

// Returns true when the command should end.
bool Shoot::IsFinished() {
  return false;
}
