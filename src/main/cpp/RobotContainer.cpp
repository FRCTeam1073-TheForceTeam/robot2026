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
// const std::string RobotContainer::testAuto = "Test Auto";

RobotContainer::RobotContainer() {
// cmd_collect = std::make_shared<Collect>(m_intake);
// m_climber = std::make_shared<Climber>();
m_drivetrain = std::make_shared<Drivetrain>();
m_OI = std::make_shared<OI>();

// m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, nullptr));
// m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_Localizer));

// m_shooterLoad = std::make_shared<ShooterLoad>();
 m_Tags = std::make_shared<AprilTagFinder>();

// m_FieldDisplay = std::make_shared<FieldMapDisplay>(m_drivetrain, m_Localizer, m_FieldMap);
 m_FieldMap = std::make_shared<FieldMap>();
// m_flywheel = std::make_shared<Flywheel>();
 m_HubFinder = std::make_shared<HubFinder>(m_Localizer);
// m_intake = std::make_shared<Intake>();
// m_laser = std::make_shared<LaserCan>();
// m_Laser = std::make_shared<LaserCan>();
 m_Localizer = std::make_shared<Localizer>(m_drivetrain, m_Tags);




  // m_drivetrain->ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)));
  // trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");
  m_ZoneFinder = std::make_shared<ZoneFinder>(m_Localizer);

  // Configure the button bindings
  ConfigureBindings();
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // TODO: un-comment this code
  // return TestAuto::Create(m_drivetrain, m_Localizer, trajectory);
}

void RobotContainer::autonomousInit() {
}

void RobotContainer::AutonomousPeriodic() {
}

void RobotContainer::ConfigureBindings() {
}
