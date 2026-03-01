// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CollectorTeleop.h"

CollectorTeleop::CollectorTeleop(std::shared_ptr<Collector>& collector, std::shared_ptr<OI>& OI) :
  m_collector(collector), 
  m_OI(OI),
  collect(false),
  last_b_button(false) {
  AddRequirements(m_collector.get());
}

// Called when the command is initially scheduled.
void CollectorTeleop::Initialize() {

  targetVelocity = 0_mps;
}

// Called repeatedly when this Command is scheduled to run
void CollectorTeleop::Execute() {  
  
 if (m_OI->GetDriverXButton()) { //for testing purposes?
    targetVelocity = -3.5_mps;
    collect = false;
  } else {
    auto b_button = m_OI->GetDriverBButton();
    if (!last_b_button && b_button) {
      collect = !collect;
    } 

    last_b_button = b_button;

    if (collect) {
      targetVelocity = 3.5_mps;
    } else {
      targetVelocity = 0_mps;
    }
  }

  m_collector->SetCommand(targetVelocity);
}

// Called once the command ends or is interrupted.
void CollectorTeleop::End(bool interrupted) {
  targetVelocity = 0_mps;
  m_collector->SetCommand(std::monostate()); // Default no-command state.
}

// Returns true when the command should end.
bool CollectorTeleop::IsFinished() {
  return false;
}
