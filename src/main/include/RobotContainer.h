// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>


#include "Constants.h"
#include <units/angle.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "subsystems/AprilTagFinder.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/FieldMap.h" 
#include "subsystems/FieldMapDisplay.h"
#include "subsystems/Flywheel.h" 
#include "subsystems/TargetFinder.h"
#include "subsystems/Collector.h"
#include "subsystems/Intake.h"  
#include "subsystems/LaserCan.h" 
#include "subsystems/Localizer.h" 
#include "subsystems/OI.h" 
#include "subsystems/ShooterHood.h" 
#include "subsystems/Turret.h"
#include "subsystems/Spindexer.h" 
#include "subsystems/ZoneFinder.h" 

#include "commands/FlywheelTeleop.h"
#include "commands/HoodTeleop.h"
#include "commands/CollectorTeleop.h"
#include "commands/IntakeTeleop.h" 
#include "commands/KickerTeleop.h"
#include "commands/TurretTeleop.h"
#include "commands/SpindexerTeleop.h"
#include "commands/TeleopDrive.h"
#include "commands/Autos.h"
#include "commands/ZeroIntake.h"
#include "commands/ZeroTurret.h"
#include "commands/ZeroHood.h"
#include "commands/ZeroClimber.h"
#include "commands/Autos/SetKicker.h"
#include "commands/Autos/SetSpindexer.h"

#include "commands/Autos.h"
#include "commands/AutoRunner.h"
#include "commands/Autos/TestAuto.h"
#include "commands/Autos/WeekZeroAuto.h"
#include "commands/Autos/BasicAuto.h"


#include <choreo/Choreo.h>
#include <utilities/ShooterTable.h>

#include "subsystems/Climber.h"
#include "commands/ClimberTeleop.h"



/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:

  // static const std::string noPosition;
  // static const std::string rightPosition;
  // static const std::string leftPosition;
  // static const std::string centerPosition;
  static const std::string weekZeroAuto;
  static const std::string testAuto;
  static const std::string noLevelAuto;
  static const std::string centerAuto;
  static const std::string basicAuto;
  static const std::string cyclicAuto;
  static const std::string eventTestAuto;
  static const std::string l_Auto;
  static const std::string basicShotAuto;
  
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  // Called from Robot
  void DisabledInit();
  bool DisabledPeriodic();

  // Called from Robot
  void TeleopInit();

 private:

  void ConfigureBindings();

  //  bool FindStartPos();

  std::shared_ptr<Drivetrain> m_drivetrain;

  std::shared_ptr<OI> m_OI;
  std::shared_ptr<FieldMap> m_fieldMap;
  std::shared_ptr<AprilTagFinder> m_tagFinder;
  std::shared_ptr<Localizer> m_localizer;
  std::shared_ptr<FieldMapDisplay> m_fieldDisplay;
  std::shared_ptr<TargetFinder> m_targetFinder;
  std::shared_ptr<ZoneFinder> m_zoneFinder;

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

  std::shared_ptr<AutoRunner> m_autoRunner;
  
  bool haveInitStartPos;

  frc::SendableChooser<std::string> m_levelChooser;

  std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory;

  // Just used for launching test commands, separate from OI and other controls.
  frc2::CommandXboxController _testController;
  frc2::CommandXboxController _operatorController;

};
