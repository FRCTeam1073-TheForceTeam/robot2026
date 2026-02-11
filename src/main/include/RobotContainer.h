// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/OI.h"
#include "subsystems/Flywheel.h"
#include "subsystems/Intake.h"
#include "commands/TeleopDrive.h"
#include "subsystems/AprilTagFinder.h"
#include "subsystems/FieldMapDisplay.h"
#include "subsystems/Localizer.h"
#include "subsystems/FieldMap.h"
#include "subsystems/LaserCan.h"
#include "commands/Collect.h"
#include "commands/Load.h"
#include "subsystems/ShooterLoad.h"
#include "subsystems/Intake.h"
#include "subsystems/Climber.h"
#include "commands/Climb.h"
#include "commands/FlywheelTeleop.h"
#include "commands/Shoot.h"
#include "commands/ClimberTeleop.h"
#include "commands/IntakeTeleop.h"
#include "commands/HoodTeleop.h"
#include "subsystems/ShooterHood.h"
#include "subsystems/ShooterLoad.h"
#include "commands/LoaderTeleop.h"
#include "subsystems/Spindexer.h"
#include "commands/SpindexerTeleop.h"
#include "commands/Laser.h"
#include "subsystems/LaserCan.h"
#include "commands/Autos/TestAuto.h"
#include <choreo/Choreo.h>
#include "subsystems/ZoneFinder.h"
#include "subsystems/HubFinder.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:

  static const std::string noPosition;
  static const std::string rightPosition;
  static const std::string leftPosition;
  static const std::string centerPosition;
  static const std::string testAuto;
  
  RobotContainer();
  // frc2::CommandPtr GetAutonomousCommand();
  bool haveInitStartPos;

  void autonomousInit();

  frc2::CommandPtr GetAutonomousCommand();

  void AutonomousPeriodic();

  frc2::Command GetTeleopCommand();
  frc2::Command GetDisbaledCommand();

  void disblaedInit();

  bool FindStartPos();

  bool DisabledPeriodic();

 private:

  std::shared_ptr<Drivetrain> m_drivetrain;
  std::shared_ptr<OI> m_OI;
  std::shared_ptr<AprilTagFinder> m_Tags;
  std::shared_ptr<FieldMapDisplay> m_FieldDisplay;
  std::shared_ptr<Localizer> m_Localizer;
  std::shared_ptr<FieldMap> m_FieldMap;
  std::shared_ptr<LaserCan> m_Laser;
  std::shared_ptr<Intake> m_intake;
  std::shared_ptr<Collect> cmd_collect;
  std::shared_ptr<ShooterLoad> m_shooterLoad;
  std::shared_ptr<Flywheel> m_flywheel;
  std::shared_ptr<Climber> m_climber;
  std::shared_ptr<ShooterHood> m_shooterHood;
  std::shared_ptr<Spindexer> m_spindexer;
  std::shared_ptr<LaserCan> m_laser;
  std::shared_ptr<TestAuto> cmd_testAuto;
  std::shared_ptr<ZoneFinder> m_ZoneFinder;
  std::shared_ptr<HubFinder> m_HubFinder;


  // std::shared_ptr<TeleopDrive> cmd_teleopDrive;

  bool isRed;

  const frc::SendableChooser<std::string> m_positionChooser;

  const frc::SendableChooser<std::string> m_levelChooser;

  std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory;

  void ConfigureBindings();

  double GearRatio = units::angle::turn_t(1)/units::angle::turn_t(1);
};
