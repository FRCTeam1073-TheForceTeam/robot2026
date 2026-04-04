// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/AprilTagFinder.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/FieldMap.h" 
#include "subsystems/FieldMapDisplay.h"
#include "subsystems/Flywheel.h" 

#include "subsystems/Collector.h"
#include "subsystems/Intake.h"  
#include "subsystems/LaserCan.h" 
#include "subsystems/Localizer.h" 
#include "subsystems/OI.h" 
#include "subsystems/ShooterHood.h" 
#include "subsystems/Turret.h"
#include "subsystems/Spindexer.h" 
#include "subsystems/ZoneFinder.h" 
#include "subsystems/Kicker.h"
#include "subsystems/Climber.h"
#include <utilities/ShooterTable.h>
#include "subsystems/TargetFinder.h"

#include "commands/DrivePath.h"
#include "commands/SmartDashPrint.h"
#include "commands/ZeroTurret.h"
#include "commands/ZeroClimber.h"
#include <frc2/command/WaitCommand.h>

#include <choreo/Choreo.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoRunner {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  explicit AutoRunner(
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
  );

  frc2::CommandPtr Create(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory, bool putIntakeOut = true);


private:
  std::shared_ptr<Drivetrain> m_drivetrain;
  std::shared_ptr<AprilTagFinder> m_Tags;
  std::shared_ptr<Localizer> m_localizer;
  std::shared_ptr<Kicker> m_kicker;
  std::shared_ptr<Climber> m_climber;
  std::shared_ptr<Flywheel> m_flywheel;
  std::shared_ptr<ShooterHood> m_shooterHood;
  std::shared_ptr<Spindexer> m_spindexer;
  std::shared_ptr<Turret> m_turret;
  std::shared_ptr<Collector> m_collector;
  std::shared_ptr<Intake> m_intake;
  std::shared_ptr<LaserCan> m_laser;
  std::shared_ptr<ShooterTable> m_shooterTable;
  std::shared_ptr<TargetFinder> m_targetFinder;

  frc2::CommandPtr Prep();

  frc2::CommandPtr PrepWithoutIntake();

  frc2::CommandPtr EventParser(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory);

  frc2::CommandPtr PartGenerator(std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory);
};