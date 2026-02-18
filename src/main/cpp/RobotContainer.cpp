// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//    !!ATTENTION!!     Most of this code is commented out because it was copied directly from weewee 2026 cpp code and 
//                      might not work. Create working code on the designated branches, and do not un-comment on main until
//                      the branch throws no errors


#include "RobotContainer.h"
#include <frc2/command/Commands.h>

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/TeleopDrive.h"
#include "commands/Collect.h"
#include "subsystems/LaserCan.h"


// const std::string RobotContainer::noPosition = "No Position";
// const std::string RobotContainer::rightPosition = "Right Auto";
// const std::string RobotContainer::leftPosition = "Left Auto";
// const std::string RobotContainer::centerPosition = "Center Auto";
// const std::string RobotContainer::testAuto = "Test Auto";

RobotContainer::RobotContainer() {
// cmd_collect = std::make_shared<Collect>(m_intake);
// m_climber = std::make_shared<Climber>();
//m_drivetrain = std::make_shared<Drivetrain>();

// Create these subsystems first!
m_OI = std::make_shared<OI>();
m_drivetrain = std::make_shared<Drivetrain>();
std::cerr << "Drivetrain created..." << std::endl;

  // m_Tags = std::make_shared<AprilTagFinder>();
  // m_FieldMap = std::make_shared<FieldMap>();
  // m_Localizer = std::make_shared<Localizer>(m_drivetrain, m_Tags);
  // m_HubFinder = std::make_shared<HubFinder>(m_Localizer);
  // m_FieldDisplay = std::make_shared<FieldMapDisplay>(m_drivetrain, m_Localizer, m_FieldMap);
  // m_FieldMap = std::make_shared<FieldMap>();
  // m_flywheel = std::make_shared<Flywheel>();
  // m_HubFinder = std::make_shared<HubFinder>(m_Localizer);
  // m_intake = std::make_shared<Intake>();
  // m_Localizer = std::make_shared<Localizer>(m_drivetrain, m_Tags);
  // m_Laser = std::make_shared<LaserCan>();
  m_spindexer = std::make_shared<Spindexer>();
  m_spindexer->SetDefaultCommand(SpindexerTeleop(m_spindexer, m_OI));
  m_flywheel = std::make_shared<Flywheel>();
  m_flywheel->SetDefaultCommand(FlywheelTeleop(m_flywheel,m_OI));

  // m_shooterLoad = std::make_shared<ShooterLoad>();
  m_climber = std::make_shared<Climber>();
  m_climber->SetDefaultCommand(ClimberTeleop(m_climber,m_OI));

  // m_drivetrain->ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)));
  // trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");
  // m_ZoneFinder = std::make_shared<ZoneFinder>(m_Localizer);
  // std::cerr << "Map, tags, localization support created..." << std::endl;

  // m_climber = std::make_shared<Climber>();
  // m_shooterLoad = std::make_shared<ShooterLoad>();

  std::cerr << "Mechanisms created..." << std::endl;


  // Configure detault commands for subsystemns:
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI).ToPtr());
  // m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_Localizer).ToPtr());

  std::cerr << "Default commands assigned..." << std::endl;
  
  // Configure the button bindings
  ConfigureBindings();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // TODO: un-comment this code
  // return TestAuto::Create(m_drivetrain, m_Localizer, trajectory);
  return frc2::cmd::Idle(); // A do-nothing command from the commands factory.
}

// Called from Robot
void RobotContainer::DisabledInit() {

}

bool RobotContainer::DisabledPeriodic() {
  return false; // TODO: Fix return value.
}

void RobotContainer::AutonomousInit() {
}

void RobotContainer::AutonomousPeriodic() {
}

void RobotContainer::ConfigureBindings() {
}
