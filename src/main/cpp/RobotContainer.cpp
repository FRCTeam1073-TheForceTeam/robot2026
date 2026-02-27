// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//    !!ATTENTION!!     Most of this code is commented out because it was copied directly from weewee 2026 cpp code and 
//                      might not work. Create working code on the designated branches, and do not un-comment on main until
//                      the branch throws no errors


#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TeleopDrive.h"
#include "commands/Collect.h"
#include "subsystems/LaserCan.h"
#include "commands/Autos/TestAuto.h"


// const std::string RobotContainer::noPosition = "No Position";
// const std::string RobotContainer::rightPosition = "Right Auto";
// const std::string RobotContainer::leftPosition = "Left Auto";
// const std::string RobotContainer::centerPosition = "Center Auto";
const std::string RobotContainer::testAuto = "Test_Auto";
const std::string RobotContainer::weekZeroAuto = "Week Zero Auto";
const std::string RobotContainer::noLevelAuto = "No Auto";

const std::string RobotContainer::noPosition = "No Position";

RobotContainer::RobotContainer() {
  // Create these subsystems first!
  m_OI = std::make_shared<OI>();
  m_drivetrain = std::make_shared<Drivetrain>();
  std::cerr << "Drivetrain created..." << std::endl;

  //m_drivetrain->ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)));
  m_FieldMap = std::make_shared<FieldMap>();
  m_Tags = std::make_shared<AprilTagFinder>();
  std::cerr << "Tags Created..." << std::endl;

  m_Localizer = std::make_shared<Localizer>(m_drivetrain, m_Tags);
  m_FieldDisplay = std::make_shared<FieldMapDisplay>(m_drivetrain, m_Localizer, m_FieldMap);
  m_HubFinder = std::make_shared<HubFinder>(m_Localizer);
  m_ZoneFinder = std::make_shared<ZoneFinder>(m_Localizer);

  std::cerr << "Localize Stuff created..." << std::endl;

  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_Localizer).ToPtr());


  // m_intake = std::make_shared<Intake>();
  // m_Laser = std::make_shared<LaserCan>();
  m_spindexer = std::make_shared<Spindexer>();
  m_spindexer->SetDefaultCommand(SpindexerTeleop(m_spindexer, m_OI).ToPtr());
  m_flywheel = std::make_shared<Flywheel>();
  m_flywheel->SetDefaultCommand(FlywheelTeleop(m_flywheel,m_OI).ToPtr());

  std::cerr << "Shoot Stuff created..." << std::endl;



// m_shooterLoad = std::make_shared<ShooterLoad>();
// m_Tags = std::make_shared<AprilTagFinder>();
  m_shooterHood = std::make_shared<ShooterHood>();
  m_shooterHood->SetDefaultCommand(HoodTeleop(m_shooterHood, m_OI).ToPtr());
  m_climber = std::make_shared<Climber>();
  m_climber->SetDefaultCommand(ClimberTeleop(m_climber,m_OI).ToPtr());

  std::cerr << "More stuff created..." << std::endl;


  // TODO: Turn on this and teleop for collector.
  m_collector = std::make_shared<Collector>();
  m_collector->SetDefaultCommand(CollectorTeleop(m_collector, m_OI).ToPtr());

  // m_drivetrain->ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)));
  // trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");
  // std::cerr << "Map, tags, localization support created..." << std::endl;
  // m_climber = std::make_shared<Climber>();
  m_kicker = std::make_shared<Kicker>();
  m_kicker->SetDefaultCommand(KickerTeleop(m_kicker,m_OI).ToPtr());

  m_turret = std::make_shared<Turret>();
  m_turret->SetDefaultCommand(TurretTeleop(m_turret, m_OI, m_HubFinder).ToPtr());

  m_laser = std::make_shared<LaserCan>();
  
  std::cerr << "Mechanisms created..." << std::endl;


  // Configure detault commands for subsystemns:
  //m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI).ToPtr());


  std::cerr << "Default commands assigned..." << std::endl;

  m_positionChooser.SetDefaultOption("No Position", noPosition);
  m_levelChooser.SetDefaultOption("No Level", noLevelAuto);
  m_levelChooser.AddOption("Week Zero Auto", weekZeroAuto);
  m_levelChooser.AddOption("Test Auto", testAuto);

  frc::SmartDashboard::PutData("Position Chooser", &m_positionChooser);
  frc::SmartDashboard::PutData("Level Chooser", &m_levelChooser);

  // Configure the button bindings
  ConfigureBindings();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // TODO: un-comment this code
  try {
    if(m_levelChooser.GetSelected() == weekZeroAuto) {
      return WeekZeroAuto::Create(m_spindexer, m_kicker, m_flywheel, m_shooterHood, m_turret);
    }
    else if (m_levelChooser.GetSelected() == testAuto) {
      trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(m_levelChooser.GetSelected()); // TODO: this will not work right now
      return TestAuto::Create(m_drivetrain, m_Localizer, trajectory);
    }
  }
  catch (...) {
    std::cerr << "Get Autonomous Command Threw Exception" << std::endl;
    return frc2::cmd::Idle(); // A do-nothing command from the commands factory.
  }
}

// Called from Robot
void RobotContainer::DisabledInit() {

}

bool RobotContainer::DisabledPeriodic() {
  return false; // TODO: Fix return value.
}

void RobotContainer::ConfigureBindings() {

}


