// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeOut.h"
#include <units/math.h>

IntakeOut::IntakeOut(std::shared_ptr<Intake> intake, bool unsafe) :
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake(intake) {
    if (!unsafe) {
      AddRequirements({m_intake.get()});
    }
}


// Called when the command is initially scheduled.
void IntakeOut::Initialize() {
  std::cerr << "Intake Out" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void IntakeOut::Execute() {
  m_intake->SetCommand(-0.1_deg);
}

// Called once the command ends or is interrupted.
void IntakeOut::End(bool interrupted) {
  
  if (interrupted) {
    std::cerr << "IntakeOut Interrupted!! " << std::endl;
  } else {
    std::cerr << "IntakeOut Finished" << std::endl;
  }
}

// Ends immediately:
bool IntakeOut::IsFinished() {
  return true;
}
