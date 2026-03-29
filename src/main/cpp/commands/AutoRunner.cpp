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
  std::shared_ptr<TargetFinder> finder
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
m_shooterTable(table)
{}


frc2::CommandPtr AutoRunner::EventParser(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  std::vector<frc2::CommandPtr> autoRoutine;

  if (trajectory.has_value()) {
    auto &traj = trajectory.value();
    auto events = traj.events;

    auto previousTime = 0_s;
  
    for(size_t e = 0; e < events.size(); e++) {
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
      else if (eventType == "Shoot") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.3_s),
              m_spindexer->SpinToSpeed(Spindexer::ShotSpeed),
              m_kicker->SpinToSpeed(Kicker::ShotSpeed),
              frc2::cmd::Wait(0.475_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.6_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(0.6_s),
              m_collector->CollectSpeed(9.14_mps),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(0.4_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(6.0_s)
        );
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_intake->IntakeOut(),
            m_flywheel->SpinToSpeed(0.0_mps),
            m_spindexer->SpinToSpeed(0.0_mps),
            m_kicker->SpinToSpeed(0.0_mps),
            m_collector->CollectSpeed(0.0_mps),
            m_shooterHood->SetHoodPosition(ShooterHood::maxPosition)
          )
        );
      }
      else if (eventType == "Shoot-Outpost") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
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
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
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
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
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
          ).WithTimeout(60.0_s)
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
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.3_s),
              m_spindexer->SpinToSpeed(6.5_mps),
              m_kicker->SpinToSpeed(6.6_mps),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(4.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(2.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(2.0_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(15_s)
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

frc2::CommandPtr AutoRunner::PartGenerator(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  std::vector<frc2::CommandPtr> parts;
  
  if (trajectory.has_value()) {
    auto &traj = trajectory.value();

    for (size_t s = 0; s < traj.splits.size(); s++) {
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

frc2::CommandPtr AutoRunner::Prep() {
  return frc2::cmd::Parallel(
    ZeroTurret(m_turret).ToPtr(),
    ZeroClimber(m_climber).ToPtr(),
    m_intake->IntakeOut()
  );//.WithTimeout(0.1_s);
}

frc2::CommandPtr AutoRunner::PrepWithoutIntake() {
  return frc2::cmd::Parallel(
    ZeroTurret(m_turret).ToPtr(),
    ZeroClimber(m_climber).ToPtr()
  ).WithTimeout(3.0_s);
}

frc2::CommandPtr AutoRunner::Create(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory, bool putIntakeOut) {

   return frc2::cmd::Sequence(
    putIntakeOut ? Prep() : PrepWithoutIntake(),
    // frc2::cmd::Wait(0.01_s),
    PartGenerator(trajectory)
  );
}
