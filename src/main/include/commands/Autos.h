// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "commands/TrackHood.h"
#include "commands/TrackTurret.h"
#include "commands/TrackFlywheel.h"
#include "subsystems/HubFinder.h"
#include "subsystems/Turret.h"
#include "subsystems/Flywheel.h"
#include "subsystems/ShooterHood.h"
#include "utilities/ShooterTable.h"


namespace Autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr TrackHub(std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<HubFinder>& hf, std::shared_ptr<ShooterTable>& st);



}  // namespace autos
