// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ExampleCommand.h"


namespace Autos {
    
frc2::CommandPtr TrackHub(std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, 
    std::shared_ptr<ShooterHood>& shooterHood, 
    std::shared_ptr<HubFinder>& hf, 
    std::shared_ptr<ShooterTable>& st){
    return frc2::cmd::Parallel(
        TrackHood(shooterHood, hf, st).ToPtr(),
        TrackFlywheel(flywheel, hf, st).ToPtr(),
        TrackTurret(turret, hf).ToPtr()
    );
}


}