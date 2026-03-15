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
{
  // Use addRequirements() here to declare subsystem dependencies.
}

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
        autoRoutine.emplace_back(m_spindexer->SpinToSpeed(5.5_mps));
      }
      else if (eventType == "StartKicker") {
        autoRoutine.emplace_back(m_kicker->SpinToSpeed(5.6_mps));
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
        autoRoutine.emplace_back(m_collector->CollectSpeed(3.5_mps +  (0.1 * m_drivetrain->GetChassisSpeeds().vx))); //TODO: maybe multiplier should be higher
      } 
      else if (eventType == "StopCollector") {
        autoRoutine.emplace_back(m_collector->CollectSpeed(0_mps));
      }
      else if (eventType == "Shoot") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.0_s),
              m_spindexer->SpinToSpeed(5.6_mps),
              m_kicker->SpinToSpeed(5.5_mps),
              frc2::cmd::Wait(5.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeOut(),
              frc2::cmd::Wait(1.0_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(12_s)
        );
      }
      else if (eventType == "Shoot-Outpost") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            Autos::TrackHub(m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.0_s),
              m_spindexer->SpinToSpeed(5.6_mps),
              m_kicker->SpinToSpeed(5.5_mps),
              frc2::cmd::Wait(6.0_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(12_s)
        );
      }
      else if (eventType == "Shoot-OutpostManual") {
        autoRoutine.emplace_back(
          frc2::cmd::Parallel(
            m_turret->RotateToPos(-140_deg),
            m_flywheel->SpinToSpeed(10.6_mps),
            m_shooterHood->RotateToPos(0.267_rad),
            frc2::cmd::Sequence(
              frc2::cmd::Wait(1.0_s),
              m_spindexer->SpinToSpeed(5.6_mps),
              m_kicker->SpinToSpeed(5.5_mps),
              frc2::cmd::Wait(6.0_s),
              m_intake->IntakeIn()
            )
          ).WithTimeout(12_s)
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

frc2::CommandPtr AutoRunner::Prep() {
  return frc2::cmd::Parallel(
    ZeroClimber(m_climber).ToPtr(),
    ZeroTurret(m_turret).ToPtr(),
    m_intake->IntakeOut()
  ).WithTimeout(3.0_s);
}

frc2::CommandPtr AutoRunner::Create(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  return frc2::cmd::Sequence(
    Prep(),
    frc2::cmd::Wait(0.5_s),
    frc2::cmd::Parallel(
      DrivePath(m_drivetrain, m_localizer, trajectory).ToPtr(),
      EventParser(trajectory)
    )
  );
}
