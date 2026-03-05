// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoRunner.h"

AutoRunner::AutoRunner(
  std::shared_ptr<Drivetrain> drivetrain,
  std::shared_ptr<FieldMap> FieldMap,
  std::shared_ptr<AprilTagFinder> Tags,
  std::shared_ptr<Localizer> Localizer,
  std::shared_ptr<FieldMapDisplay> FieldDisplay,
  std::shared_ptr<HubFinder> HubFinder,
  std::shared_ptr<ZoneFinder> ZoneFinder,
  std::shared_ptr<Kicker> kicker,
  std::shared_ptr<Climber> climber,
  std::shared_ptr<Flywheel> flywheel,
  std::shared_ptr<ShooterHood> shooterHood,
  std::shared_ptr<Spindexer> spindexer,
  std::shared_ptr<Turret> turret,
  std::shared_ptr<Collector> collector,
  std::shared_ptr<Intake> intake,
  std::shared_ptr<LaserCan> laser,
  std::shared_ptr<ShooterTable> shooterTable,
  std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory  
) :
m_drivetrain(drivetrain),
m_FieldMap(FieldMap),
m_Tags(Tags),
m_Localizer(Localizer),
m_FieldDisplay(FieldDisplay),
m_HubFinder(HubFinder),
m_ZoneFinder(ZoneFinder),
m_kicker(kicker),
m_climber(climber),
m_flywheel(flywheel),
m_shooterHood(shooterHood),
m_spindexer(spindexer),
m_turret(turret),
m_collector(collector),
m_intake(intake),
m_laser(laser),
m_shooterTable(shooterTable),
trajectory(trajectory)
{
  // Use addRequirements() here to declare subsystem dependencies.
}

std::vector<std::unique_ptr<frc2::Command>> AutoRunner::EventListener() {
  std::vector<std::unique_ptr<frc2::Command>> autoSequence;

  if (trajectory.has_value()) {
    auto &traj = trajectory.value();
    auto events = traj.events;

    autoSequence.emplace_back(frc2::cmd::Print("Starting"));
    autoSequence.emplace_back(DrivePath(m_drivetrain, m_Localizer, trajectory));

    std::vector<std::unique_ptr<frc2::Command>> parallelSequence; // might not be unique pointers or may need to convert
    for(int e = 0; e < events.size(); e++) {
      auto activeEvent = events.at(e);
      auto eventType = activeEvent.event;

      //TODO: discuss with Strategy subgroup what we will call this
      if (eventType == "StartFlywheel") {
        parallelSequence.emplace_back(m_flywheel->SpinToSpeed(14_mps));
      }
      else if (eventType == "StartSpindexer") {
        parallelSequence.emplace_back(m_spindexer->SpinToSpeed(4.2_mps));
      } //TODO: continue with other events
      else if(eventType.substr(0,4) == "Stop") {
        auto waitTime = activeEvent.timestamp - (events.at(e - 1).timestamp);
        parallelSequence.emplace_back(frc2::cmd::Wait(waitTime));

        //TODO: do things for stop here
      }
    }
    autoSequence.emplace_back(parallelSequence);
  }
}

frc2::CommandPtr AutoRunner::Create() {
  return frc2::cmd::Sequence(); //TODO: put in eventlistener as a call
}
