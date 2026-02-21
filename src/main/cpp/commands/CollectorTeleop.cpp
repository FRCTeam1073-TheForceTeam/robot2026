// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CollectorTeleop.h"

CollectorTeleop::CollectorTeleop(std::shared_ptr<Collector> collector, std::shared_ptr<OI> OI) :
  m_collector(collector), 
  m_OI(OI) {
  spin = false; 
  AddRequirements(m_collector.get());
}

// Called when the command is initially scheduled.
void CollectorTeleop::Initialize() {
  isMoving = false;
  targetVelocity = 0_mps;
}

// Called repeatedly when this Command is scheduled to run
void CollectorTeleop::Execute() {  

  // TODO: Button mappings.
  //
  // if (m_OI->GetOperatorAButton()){
  //   targetVelocity = 1.5_mps;
  //   isMoving = true;
  // }
  // else if (m_OI->GetOperatorBButton()) {
  //   targetVelocity = -1.5_mps;
  //   isMoving = true;  
  // }
  // else{
  //   targetVelocity = 0_mps;
  //   isMoving = false;
  // }

  // m_collector->SetCommand(targetVelocity);
  frc::SmartDashboard::PutBoolean("CollectorTeleop/AButton", m_OI->GetOperatorAButton());
  frc::SmartDashboard::PutBoolean("CollectorTeleop/Is Moving", isMoving);
}

// Called once the command ends or is interrupted.
void CollectorTeleop::End(bool interrupted) {
  targetVelocity = 0_mps;
  isMoving = false;
  m_collector->SetCommand(std::monostate()); // Default no-command state.
}

// Returns true when the command should end.
bool CollectorTeleop::IsFinished() {
  return false;
}
