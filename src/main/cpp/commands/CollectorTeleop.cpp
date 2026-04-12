// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CollectorTeleop.h"

CollectorTeleop::CollectorTeleop(std::shared_ptr<Collector>& collector, std::shared_ptr<OI>& OI, std::shared_ptr<Drivetrain>& dt) :
  m_collector(collector), 
  m_OI(OI),
  m_dt(dt),
  collect(false),
  last_trigger(false) {

    // DO NOT REQUIRE DRIVETRAIN:
  AddRequirements(m_collector.get());
}

// Called when the command is initially scheduled.
void CollectorTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CollectorTeleop::Execute() {  

  // TODO: ASK SILLY DIRIVE TEAM!!
 if (std::abs(m_OI->GetDriverAButton()) >= 0.1) { // To eject fuel.
    m_collector->SetCommand(-4.0_mps);
  }
  else if (std::abs(m_OI->GetDriverRightTrigger()) >= 0.1) {
    //m_collector->SetCommand(3.5_mps + (0.1 * m_dt->GetChassisSpeeds().vx));
    m_collector->SetCommand(9.14_mps);
  }
  else {
    m_collector->SetCommand(std::monostate());
  }

  
}

// Called once the command ends or is interrupted.
void CollectorTeleop::End(bool interrupted) { 
  m_collector->SetCommand(std::monostate()); // Default no-command state.
}

// Returns true when the command should end.
bool CollectorTeleop::IsFinished() {
  return false;
}
