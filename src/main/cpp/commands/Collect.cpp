// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Collect.h"
#include <frc2/command/Command.h>

Collect::Collect(std::shared_ptr<Intake> Intake) :
    m_intake{Intake} {
    AddRequirements({m_intake.get()});
}

void Collect::Initialize() {
    m_intake->SetIntakeVelocity(1.0_tps);
}

void Collect::Execute() {
    m_intake->SetIntakeVelocity(1.0_tps);
}

void Collect::End(bool interrupted) {
    m_intake->SetIntakeVelocity(0.0_tps);
}

bool Collect::IsFinished() {
    return false;
}