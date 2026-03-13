// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/TargetFinder.h"
#include "subsystems/Turret.h"
#include "subsystems/Flywheel.h"
#include "subsystems/ShooterHood.h"
#include "subsystems/Kicker.h"
#include "subsystems/Spindexer.h"
#include "utilities/ShooterTable.h"


namespace Autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr TrackHub(std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st);
frc2::CommandPtr BasicAutoShot(std::shared_ptr<Spindexer>& spindexer, std::shared_ptr<Kicker>& kicker, std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st);
frc2::CommandPtr HubAuto(std::shared_ptr<Spindexer>& spindexer, std::shared_ptr<Kicker>& kicker, std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<ShooterHood>& shooterHood);

}  // namespace Autos
