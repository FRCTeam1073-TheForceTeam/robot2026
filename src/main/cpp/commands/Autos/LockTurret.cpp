#include "commands/Autos/LockTurret.h"
#include <iostream>
#include <frc/DriverStation.h>


LockTurret::LockTurret(std::shared_ptr<Turret>& turret, std::shared_ptr<TargetFinder>& targetFinder, std::shared_ptr<AprilTagFinder>& aprilTag) :
  m_turret(turret),
  m_targetFinder(targetFinder), 
  m_aprilTag(aprilTag)
{
  lastError = 0,
  isAlignedToHub = false,
  m_targetPosition = 0_rad,
  position = 0_rad,
  AddRequirements(m_turret.get());
}

// Called when the command is initially scheduled.
void LockTurret::Initialize() 
{
  std::cerr << "LockTurret Init" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void LockTurret::Execute() 
{
  m_targetPosition = m_targetFinder->getFeedback().turretToTargetAngle;
  m_turret->SetCommand(m_targetPosition);

  if (m_targetFinder->getFeedback().target == "Hub" && m_aprilTag->getTurretResults().size() != 0)
  {
    auto TurretCam = m_aprilTag->getTurretResults()[0]._pose;
    m_targetPosition = TurretCam.RelativeTo(m_targetFinder->OurHub).Rotation().Radians();
    m_turret->SetCommand(m_targetPosition);
  }

  frc::SmartDashboard::PutNumber("Turret/position", position.value());
  frc::SmartDashboard::PutNumber("Turret/targetPosition", m_targetPosition.value());
}

// Called once the command ends or is interrupted.
void LockTurret::End(bool interrupted) 
{
  if (interrupted) {
        std::cerr << "LockTurret: Interrupted!" << std::endl;
    }
    m_turret->SetCommand(std::monostate());
}

// Returns true when the command should end.
bool LockTurret::IsFinished() 
{
  return false;
}
