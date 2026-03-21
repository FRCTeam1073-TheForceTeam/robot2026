// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>

#include "commands/Autos/TrackHood.h"
#include "commands/Autos/TrackTurret.h"
#include "commands/Autos/TrackFlywheel.h"
#include "commands/Autos/SetKicker.h"
#include "commands/Autos/SetSpindexer.h"

namespace Autos {
    
frc2::CommandPtr TrackHub(std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, 
    std::shared_ptr<ShooterHood>& shooterHood, 
    std::shared_ptr<TargetFinder>& hf, 
    std::shared_ptr<ShooterTable>& st){
    return frc2::cmd::Parallel(
        TrackHood(shooterHood, hf, st).ToPtr(),
        TrackFlywheel(flywheel, hf, st).ToPtr(),
        TrackTurret(turret, hf).ToPtr()
    );
}

frc2::CommandPtr BasicAutoShot(std::shared_ptr<Spindexer>& spindexer, std::shared_ptr<Kicker>& kicker, std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st){
    return
        frc2::cmd::Parallel(
        TrackHood(shooterHood, hf, st).ToPtr(),
        TrackFlywheel(flywheel, hf, st).ToPtr(),
        TrackTurret(turret, hf).ToPtr(),
        frc2::cmd::Sequence(frc2::cmd::Wait(2.0_s), 
        frc2::cmd::Parallel(
        SetSpindexer(spindexer).ToPtr(),
        SetKicker(kicker).ToPtr()))).WithTimeout(10.0_s);
}
frc2::CommandPtr HubAuto(std::shared_ptr<Spindexer>& spindexer, std::shared_ptr<Kicker>& kicker, std::shared_ptr<Turret>& turret, std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<ShooterHood>& shooterHood){
    return
        frc2::cmd::Parallel(
        shooterHood->SetPosition(69.2_deg),
        flywheel->SpinToSpeed(8_mps),
        turret->RotateToPos(0_rad),
        frc2::cmd::Sequence(frc2::cmd::Wait(2.0_s), 
        frc2::cmd::Parallel(
        SetSpindexer(spindexer).ToPtr(),
        SetKicker(kicker).ToPtr()))).WithTimeout(10.0_s);
}

}