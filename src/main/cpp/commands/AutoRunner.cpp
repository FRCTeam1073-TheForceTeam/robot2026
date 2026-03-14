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
        autoRoutine.emplace_back(m_spindexer->SpinToSpeed(4.2_mps));
      }
      else if (eventType == "StartKicker") {
        autoRoutine.emplace_back(m_kicker->SpinToSpeed(4.5_mps));
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
        autoRoutine.emplace_back(m_collector->CollectSpeed(-3.5_mps));
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
              m_spindexer->SpinToSpeed(4.2_mps),
              m_kicker->SpinToSpeed(4.5_mps),
              frc2::cmd::Wait(4.0_s),
              m_intake->IntakeIn(),
              frc2::cmd::Wait(4_s),
              m_spindexer->SpinToSpeed(0.0_mps),
              m_kicker->SpinToSpeed(0.0_mps)
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

frc2::CommandPtr AutoRunner::PartGenerator(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  std::vector<frc2::CommandPtr> parts;
  
  if (trajectory.has_value()) {
    auto &traj = trajectory.value();

    for (int s = 0; s < traj.splits.size(); s++) {
      auto split_traj = traj.GetSplit(s);

      auto part = frc2::cmd::Parallel(
        DrivePath(m_drivetrain, m_localizer, split_traj).ToPtr(),
        EventParser(split_traj)
      );
      parts.emplace_back(SmartDashPrint("start of partts, about to move(part)"));
      parts.emplace_back(std::move(part));
      parts.emplace_back(SmartDashPrint("just moved, about to print the ToPtr() thing"));
      parts.emplace_back(SmartDashPrint(std::to_string(s)).ToPtr());
      parts.emplace_back(SmartDashPrint("about to wait one second"));
      parts.emplace_back(frc2::cmd::Wait(1_s));
      parts.emplace_back(SmartDashPrint("done waiting"));
    }

    return frc2::cmd::Sequence(std::move(parts));
  }
}

frc2::CommandPtr AutoRunner::Prep() {
  return frc2::cmd::Sequence(
    ZeroTurret(m_turret).ToPtr(),
    m_intake->IntakeOut(),
    frc2::cmd::Wait(0.5_s)
  );
}

frc2::CommandPtr AutoRunner::Create(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
   return frc2::cmd::Sequence(
    Prep(),
    PartGenerator(trajectory)
  );
}
