#include "commands/Autos/LockTurret.h"
#include <iostream>
#include <frc/DriverStation.h>


LockTurret::LockTurret(std::shared_ptr<Turret>& turret, std::shared_ptr<TargetFinder>& targetFinder) :
  m_turret(turret),
  m_targetFinder(targetFinder) 
  {
  lastError = 0,
  isAlignedToHub = false,
  targetPosition = 0_rad,
  position = 0_rad,
  AddRequirements(m_turret.get());
}

// Called when the command is initially scheduled.
void LockTurret::Initialize() {
  std::cerr << "TrackTurret Init" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void LockTurret::Execute() {
  targetPosition = m_targetFinder->getFeedback().turretToTargetAngle;
  m_turret->SetCommand(targetPosition);

  

  frc::SmartDashboard::PutNumber("Turret/position", position.value());
  frc::SmartDashboard::PutNumber("Turret/targetPosition", targetPosition.value());
}

// Called once the command ends or is interrupted.
void LockTurret::End(bool interrupted) {
  if (interrupted) {
        std::cerr << "TrackTurret: Interrupted!" << std::endl;
    }
    m_turret->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool LockTurret::IsFinished() {
  return false;
}
