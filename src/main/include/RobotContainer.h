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
#include "commands/TeleopDrive.h"
#include "subsystems/AprilTagFinder.h"
#include "subsystems/FieldMapDisplay.h"
#include "subsystems/Localizer.h"
#include "subsystems/FieldMap.h"

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

  frc2::Command GetAutonomousCommand();
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


  // std::shared_ptr<TeleopDrive> cmd_teleopDrive;

  bool isRed;

  const frc::SendableChooser<std::string> m_positionChooser;


  const frc::SendableChooser<std::string> m_levelChooser;


  void ConfigureBindings();
};
