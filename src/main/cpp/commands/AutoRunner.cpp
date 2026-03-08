// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoRunner.h"

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
  std::shared_ptr<LaserCan> laser
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
m_laser(laser)
{
  // Use addRequirements() here to declare subsystem dependencies.
}

std::vector<frc2::CommandPtr> AutoRunner::EventListener(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
  std::vector<frc2::CommandPtr> autoRoutine;

  if (trajectory.has_value()) {
    auto &traj = trajectory.value();
    auto events = traj.events;

    autoRoutine.emplace_back(frc2::cmd::Print("Starting"));
    autoRoutine.emplace_back(DrivePath(m_drivetrain, m_localizer, trajectory).ToPtr());
  
    std::vector<frc2::CommandPtr> parallelSequence;
    for(int e = 0; e < events.size(); e++) {
      auto activeEvent = events.at(e);
      auto eventType = activeEvent.event;

      //TODO: discuss with Strategy subgroup what we will call this
      if (eventType == "StartFlywheel") {
        parallelSequence.emplace_back(m_flywheel->SpinToSpeed(14_mps));
      }
      else if (eventType == "StartSpindexer") {
        parallelSequence.emplace_back(m_spindexer->SpinToSpeed(4.2_mps));
      }
      else if (eventType == "StartKicker") {
        parallelSequence.emplace_back(m_kicker->SpinToSpeed(4.5_mps));
      }
      else if (eventType.substr(0, 12) == "SetHoodLevel") {
        parallelSequence.emplace_back(m_shooterHood->SetHoodLevel(0));
      }
      else if (eventType == "SetTurret") {
         parallelSequence.emplace_back(m_turret->RotateToPos(90_deg));
      }
      else if (eventType == "IntakeOut") {
        parallelSequence.emplace_back(m_intake->IntakeOut());
      }
      else if (eventType == "IntakeIn") {
        parallelSequence.emplace_back(m_intake->IntakeIn());
      }
      else if (eventType == "StartCollector") {
        parallelSequence.emplace_back(m_collector->CollectSpeed(3.5_mps));
      }
      else if (eventType.substr(0, 4) == "Wait") {
        parallelSequence.emplace_back(frc2::cmd::Wait(units::second_t(eventType[5])));
      }
  //     //TODO: put in other complex shooter commands
      else if(eventType.substr(0,4) == "Stop") {
        auto waitTime = activeEvent.timestamp - (events.at(e - 1).timestamp);
        parallelSequence.emplace_back(frc2::cmd::Wait(waitTime));
        if (eventType == "StopFlywheel") {
          parallelSequence.emplace_back(m_flywheel->SpinToSpeed(0_mps));
        }
        else if (eventType == "StopSpindexer") {
          parallelSequence.emplace_back(m_spindexer->SpinToSpeed(0_mps));
        }
        else if (eventType == "StopKicker") {
          parallelSequence.emplace_back(m_kicker->SpinToSpeed(0_mps));
        }
        else if (eventType == "StopCollector") {
          parallelSequence.emplace_back(m_collector->CollectSpeed(0_mps));
        }
      }
    }

    autoRoutine.emplace_back(
      frc2::cmd::Sequence(std::move(parallelSequence))
    );
    // autoRoutine.emplace_back(m_kicker->SpinToSpeed(1_mps));

  }
  return autoRoutine;
}

frc2::CommandPtr AutoRunner::Create(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) {
   return frc2::cmd::Parallel(EventListener(trajectory));
   //TODO: think about mirroring for red
}
