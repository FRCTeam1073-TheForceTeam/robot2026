// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoRunner.h"
#include "commands/Autos.h"

AutoRunner::AutoRunner(
  std::shared_ptr<Drivetrain> drivetrain,
  std::shared_ptr<AprilTagFinder> Tags,
  std::shared_ptr<Localizer> localizer,
  std::shared_ptr<Kicker> kicker,
  std::shared_ptr<Climber> climber,
  std::shared_ptr<Flywheel> flywheel,
  std::shared_ptr<ShooterHood> shooterHood,
  std::shared_ptr<Spindexer> spindexer,
  std::shared_ptr<Turret> turret,
  std::shared_ptr<Collector> collector,
  std::shared_ptr<Intake> intake,
  std::shared_ptr<LaserCan> laser,
  std::shared_ptr<ShooterTable> table,
  std::shared_ptr<TargetFinder> finder,
  std::shared_ptr<Bling> bling,
  std::shared_ptr<BallisticShot> bs
) :
m_drivetrain(drivetrain),
m_Tags(Tags),
m_localizer(localizer),
m_kicker(kicker),
m_climber(climber),
m_flywheel(flywheel),
m_shooterHood(shooterHood),
m_spindexer(spindexer),
m_turret(turret),
m_collector(collector),
m_intake(intake),
m_laser(laser),
m_targetFinder(finder),
m_shooterTable(table),
m_bling(bling),
m_bs(bs),
// Create prep commands: Unsafe versionsjust run in parallel with no requirements!!
m_zeroTurret(ZeroTurret(m_turret, true).ToPtr()),     // Unsafe version of command.
m_zeroClimber(ZeroClimber(m_climber, true).ToPtr()),  // Unsafe version of command.
m_intakeOut(IntakeOut(m_intake, true).ToPtr())       // Unsafe version of command.
{


}


frc2::CommandPtr AutoRunner::EventParser(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  std::vector<frc2::CommandPtr> autoRoutine;

  if (trajectory.has_value()) {
    auto &traj = trajectory.value();
    auto events = traj.events;

    auto previousTime = 0_s;
  
    for(int e = 0; e < events.size(); e++) {
      auto activeEvent = events.at(e);
      auto eventType = activeEvent.event;

      auto waitTime = activeEvent.timestamp - previousTime;
      autoRoutine.emplace_back(frc2::cmd::Wait(waitTime));
      autoRoutine.emplace_back(SmartDashPrint(eventType).ToPtr());

      previousTime = activeEvent.timestamp;
      if (eventType == "StartSpindexer") {
        autoRoutine.emplace_back(m_spindexer->SpinToSpeed(Spindexer::ShotSpeed));
      }
      else if (eventType == "StartKicker") {
        autoRoutine.emplace_back(m_kicker->SpinToSpeed(Kicker::ShotSpeed));
      }
      else if (eventType == "StopSpindexer") {
        autoRoutine.emplace_back(m_spindexer->SpinToSpeed(0_mps));
      }
      else if (eventType == "StopKicker") {
        autoRoutine.emplace_back(m_kicker->SpinToSpeed(0_mps));
      }
      else if (eventType == "DeployIntake") {
        autoRoutine.emplace_back(m_intake->IntakeOut());
      }
      else if (eventType == "RetractIntake") {
        autoRoutine.emplace_back(m_intake->IntakeIn());
      } 
      else if (eventType == "StartCollector") {
        autoRoutine.emplace_back(m_collector->CollectSpeed(9.14_mps)); //TODO: maybe multiplier should be higher
      } 
      else if (eventType == "StopCollector") {
        autoRoutine.emplace_back(m_collector->CollectSpeed(0_mps));
      }
      else if (eventType == "ZeroTurret") {
        autoRoutine.emplace_back(ZeroTurret(m_turret).ToPtr());
      }
      else if (eventType == "ZeroClimber") {
        autoRoutine.emplace_back(ZeroClimber(m_climber).ToPtr());
      }
      else if (eventType == "BringUpClimber") {
        autoRoutine.emplace_back(m_climber->ClimberPosition(0.0582_m));
      }
      else if (eventType == "BringDownClimber") {
        autoRoutine.emplace_back(
          frc2::cmd::Sequence(
            m_climber->HoldVelocity(-0.67_mps),
            frc2::cmd::Wait(0.1_s),
            m_climber->HoldVelocity(0.0_mps)
          )
        );
      }
      else if (eventType == "TurretRotation125") {
        autoRoutine.emplace_back(m_turret->RotateToPos(125_deg));
      }
      else if (eventType == "TurretRotation90") {
        autoRoutine.emplace_back(m_turret->RotateToPos(90_deg));
      }
      else if (eventType == "TurretRotation70") {
        autoRoutine.emplace_back(m_turret->RotateToPos(70_deg));
      }
      // else if (eventType == "TurretRotaton(-150)") {
      //   autoRoutine.emplace_back(m_turret->RotateToPos(-150_deg));
      // }
      else if (eventType == "Shoot") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.5_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(5.5_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_intake->IntakeOut(),
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0.0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "Shoot-Outpost") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.0_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(6.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(2.0_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(15_s)
        );
      }
      else if (eventType == "Shoot-OutpostManual") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_turret->RotateToPos(-140_deg),
            m_flywheel->SpinToSpeed(10.5_mps),
            m_shooterHood->SetHoodPosition(0.267_rad),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.0_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(6.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(2.0_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(15_s)
        );
      }
      else if (eventType == "CenterShoot") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.5_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut()
              
            )
          ).WithTimeout(6.75_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "CenterShootOutpost") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.85_s),
              m_spindexer->SpinToSpeed(6.5_mps),
              m_kicker->SpinToSpeed(6.6_mps),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(2.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.3_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut()
            )
          ).WithTimeout(6.0_s)//TODO: find the optimal timeout for the autos
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "CenterDepotToOutpostShoot") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.7_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s)
            )
          ).WithTimeout(3.8_s)//TODO: find the optimal timeout for the autos
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "Pause") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            frc2::cmd::Sequence()
          ).WithTimeout(0.5_s)//TODO: find the optimal timeout for the autos
        );
      }
      else if (eventType == "CenterOutpostToClimbShoot") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.0_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(4.2_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "ShootMovingOutpost") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.3_s),
              m_spindexer->SpinToSpeed(6.5_mps),
              m_kicker->SpinToSpeed(6.6_mps),
              frc2::cmd::Wait(0.5_s),
              m_bling->blingPurple(), //purple: about to go in
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              m_bling->blingWhite(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(3.5_s),
              m_bling->blingPurple(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              m_bling->blingWhite(),
              frc2::cmd::Wait(2.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(1.5_s),
              m_bling->blingPurple(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              m_bling->blingWhite()
            )
          ).WithTimeout(15_s)
        );
      }
      else if (eventType == "ShootDoublePath") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.3_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed)
            )
          ).WithTimeout(5_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "ShootBumpAuto") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              m_collector->CollectSpeed(0.0_mps),
              frc2::cmd::Wait(1.2_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(10_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      } 
      else if (eventType == "ShootBump") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.2_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.2_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(5.4_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_intake->IntakeOut(),
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0.0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition),
            m_collector->CollectSpeed(9.14_mps)
          )
        ); 
      }  
      else if (eventType == "ShootFollow") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.5_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(5.0_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "ShootLeftFollow") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.5_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeOut()
            )
          ).WithTimeout(5.2_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }  
      else if (eventType == "ShootLeftFollow") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_bs),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(0.5_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.5_s),
              m_climber->ClimberPosition(0.0582_m),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.5_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(2.5_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition),
            m_intake->IntakeIn()
          )
        );
      }   
    }

    return frc2::cmd::Sequence(std::move(autoRoutine));
  }
  else {
    std::cerr << "Error: returning Idle Command" << std::endl;
    return frc2::cmd::Idle();
  }  
}

frc2::CommandPtr AutoRunner::PartGenerator(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory, units::time::second_t delay) {
  std::vector<frc2::CommandPtr> parts;
  
  if (delay > 0.0_s) {
    parts.emplace_back(std::move(frc2::cmd::Wait(delay))); // If we have a delay add it to the 1st part.
  }

  if (trajectory.has_value()) {
    auto &traj = trajectory.value();

    for (int s = 0; s < traj.splits.size(); s++) {
      auto split_traj = traj.GetSplit(s);

      auto part = frc2::cmd::Parallel(
        DrivePath(m_drivetrain, m_localizer, split_traj).ToPtr(),
        EventParser(split_traj)
      );
      
      parts.emplace_back(std::move(part));
    }

    return frc2::cmd::Sequence(std::move(parts));
  }
  else {
    std::cerr << "Auto Runner Part Generator not have a trajectory" << std::endl;
    SmartDashPrint("No Trajectory");
    return frc2::cmd::Idle(); // You have to return something!?
  }
}

frc2::CommandPtr AutoRunner::Prep(units::time::second_t delay) {
  return frc2::cmd::Parallel(
    frc2::cmd::Wait(delay + 0.01_s),
    frc2::ScheduleCommand(m_zeroTurret.get()).ToPtr(),  // Unsafe command
    frc2::ScheduleCommand(m_zeroClimber.get()).ToPtr(), // Unsafe command
    frc2::ScheduleCommand(m_intakeOut.get()).ToPtr()    // Unsafe command
  ).WithTimeout(5.0_s); // Absolute maximum time...
}

frc2::CommandPtr AutoRunner::PrepWithoutIntake(units::time::second_t delay) {
  return frc2::cmd::Parallel(
      frc2::cmd::Wait(delay + 0.01_s),
    frc2::ScheduleCommand(m_zeroTurret.get()).ToPtr(), // Unsafe command  
    frc2::ScheduleCommand(m_zeroClimber.get()).ToPtr() // Unsafe command
    ).WithTimeout(5.0_s); // Absolute maximum time...
}

frc2::CommandPtr AutoRunner::Create(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory, units::time::second_t start_delay, bool putIntakeOut) {
  auto partsList = PartGenerator(trajectory, start_delay);

  return frc2::cmd::Sequence(
    putIntakeOut ? Prep(start_delay) : PrepWithoutIntake(start_delay),
    frc2::cmd::Sequence(std::move(partsList))
  ).WithTimeout(30.0_s); // Absolute maximumtime... real auto is 20s.
}
