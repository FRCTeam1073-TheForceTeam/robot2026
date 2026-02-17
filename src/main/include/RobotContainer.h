// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//    !!ATTENTION!!     Most of this code is commented out because it was copied directly from weewee 2026 cpp code and 
//                      might not work. Create working code on the designated branches, and do not un-comment on main until
//                      the branch throws no errors









#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include <units/angle.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/AprilTagFinder.h"  // Unfinished
#include "subsystems/Climber.h"// Unfinished
#include "subsystems/DriveTrain.h"// Unfinished
#include "subsystems/ExampleSubsystem.h"// Unfinished
#include "subsystems/FieldMap.h"// Unfinished
#include "subsystems/FieldMapDisplay.h"// Unfinished
#include "subsystems/Flywheel.h"// Unfinished
#include "subsystems/HubFinder.h"// Unfinished
#include "subsystems/Intake.h"// Unfinished
#include "subsystems/LaserCan.h"// Unfinished
#include "subsystems/Localizer.h"// Unfinished
#include "subsystems/OI.h"// Unfinished
#include "subsystems/ShooterHood.h"// Unfinished
#include "subsystems/ShooterLoad.h"// Unfinished
#include "subsystems/Spindexer.h"// Unfinished
#include "subsystems/ZoneFinder.h"// Unfinished

#include "commands/Climb.h"// Unfinished
#include "commands/ClimberTeleop.h"// Unfinished
#include "commands/Collect.h"// Unfinished
#include "commands/FlywheelTeleop.h"// Unfinished
#include "commands/HoodTeleop.h"// Unfinished
#include "commands/IntakeTeleop.h"// Unfinished
#include "commands/Laser.h"// Unfinished
#include "commands/Load.h"// Unfinished
#include "commands/LoaderTeleop.h"// Unfinished
#include "commands/Shoot.h"// Unfinished
#include "commands/SpindexerTeleop.h"// Unfinished
#include "commands/TeleopDrive.h"// Unfinished

#include "commands/Autos/TestAuto.h"// Unfinished

#include <choreo/Choreo.h>// Unfinished


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
  // static const std::string testAuto;
  
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();

  // Called from Robot
  void AutonomousInit();
  void AutonomousPeriodic();

  // Called from Robot
  frc2::Command GetTeleopCommand();

  // Called from Robot
  void DisabledInit();
  bool DisabledPeriodic();


 private:

  //  bool FindStartPos();

   std::shared_ptr<Drivetrain> m_drivetrain;
   std::shared_ptr<OI> m_OI;
  // std::shared_ptr<AprilTagFinder> m_Tags;
  // std::shared_ptr<FieldMapDisplay> m_FieldDisplay;
  // std::shared_ptr<Localizer> m_Localizer;
  // std::shared_ptr<FieldMap> m_FieldMap;
  // std::shared_ptr<LaserCan> m_Laser;
  // std::shared_ptr<Intake> m_intake;
  // std::shared_ptr<Collect> cmd_collect;
  // std::shared_ptr<ShooterLoad> m_shooterLoad;
  // std::shared_ptr<Flywheel> m_flywheel;
  // std::shared_ptr<Climber> m_climber;
  // std::shared_ptr<ShooterHood> m_shooterHood;
  std::shared_ptr<Spindexer> m_spindexer;
  // std::shared_ptr<LaserCan> m_laser;
  // std::shared_ptr<TestAuto> cmd_testAuto;
  // std::shared_ptr<ZoneFinder> m_ZoneFinder;
  // std::shared_ptr<HubFinder> m_HubFinder;

  bool haveInitStartPos;

  std::shared_ptr<Spindex> cmd_zspindex;
  std::shared_ptr<SpindexerTeleop> cmd_zspindexerTeleop;

  // std::shared_ptr<TeleopDrive> cmd_teleopDrive;

  bool isRed;

  const frc::SendableChooser<std::string> m_positionChooser;

  const frc::SendableChooser<std::string> m_levelChooser;

  // std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory;

  void ConfigureBindings();
};
