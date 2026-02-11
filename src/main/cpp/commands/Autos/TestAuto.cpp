// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/TestAuto.h"

// TestAuto::TestAuto(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory)
// {
//   TestAuto::path_trajectory = trajectory;
//   TestAuto::m_drivetrain = drivetrain;
//   TestAuto::m_localizer = localizer;
//   AddRequirements({m_drivetrain.get(), m_localizer.get()});
// }

frc2::CommandPtr TestAuto::Create(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  return frc2::CommandPtr(DrivePath(drivetrain, localizer, trajectory));
}
