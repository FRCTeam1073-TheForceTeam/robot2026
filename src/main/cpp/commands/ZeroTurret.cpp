// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroTurret.h"
#include <units/math.h>

ZeroTurret::ZeroTurret(std::shared_ptr<Turret> turret, bool unsafe) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_turret(turret), 
  limit(3.5_Nm)  {
    if (!unsafe) {
      AddRequirements({m_turret.get()});
    }
}


// Called when the command is initially scheduled.
void ZeroTurret::Initialize() {
  std::cerr << "Zero Turret" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void ZeroTurret::Execute() {
  auto velocity = 2.0_rad_per_s; //TODO: change this value
  m_turret->SetCommand(velocity);
}

// Called once the command ends or is interrupted.
void ZeroTurret::End(bool interrupted) {
  
  if (interrupted) {
    std::cerr << "ZeroTurret Interrupted!! " << std::endl;
  } else {
    std::cerr << "ZeroTurret Finished" << std::endl;
  }
  m_turret->Zero();
  m_turret->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool ZeroTurret::IsFinished() {
  if(units::math::abs(m_turret->GetFeedback().torque) > limit) { //TODO: change limit
    return true;
  }
  return false;
}
