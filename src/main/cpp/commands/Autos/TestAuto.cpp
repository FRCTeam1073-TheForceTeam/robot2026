// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/TestAuto.h"

frc2::CommandPtr TestAuto::Create(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  return DrivePath(drivetrain, localizer, trajectory).ToPtr();
}
