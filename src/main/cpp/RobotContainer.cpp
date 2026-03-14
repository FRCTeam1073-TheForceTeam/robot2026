// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TeleopDrive.h"
#include "subsystems/LaserCan.h"
#include "commands/Autos/TestAuto.h"

const std::string RobotContainer::weekZeroAuto = "Week Zero Auto";
const std::string RobotContainer::noLevelAuto = "No Auto";
const std::string RobotContainer::basicAuto = "Basic Auto";
const std::string RobotContainer::basicShotAuto = "Basic Shot Auto";
const std::string RobotContainer::exampleAuto = "Example_Auto";

const std::string RobotContainer::neutralRightTrench = "NeutralRightTrench";
const std::string RobotContainer::neutralLeftTrench = "NeutralLeftTrench";
const std::string RobotContainer::halfNeutralRight = "HalfNeutralRight";
const std::string RobotContainer::halfNeutralLeft = "HalfNeutralLeft";
const std::string RobotContainer::doubleNeutralRight = "DoubleNeutralRight";

const std::string RobotContainer::splitAuto = "SplitAuto";
const std::string RobotContainer::cornerShotAuto = "CornerShotAuto";

const std::string RobotContainer::hubAuto = "Hub Auto";


RobotContainer::RobotContainer() :
_operatorController(1)
{
  // Create these subsystems first!
  m_OI = std::make_shared<OI>();
  m_drivetrain = std::make_shared<Drivetrain>();

  std::cerr << "\tDrivetrain created..." << std::endl;

  // Must be here because localizer depends on this due to moving camera.
  m_turret = std::make_shared<Turret>();

  m_fieldMap = std::make_shared<FieldMap>();
  m_tagFinder = std::make_shared<AprilTagFinder>(m_turret,m_drivetrain);
  m_localizer = std::make_shared<Localizer>(m_drivetrain, m_tagFinder);
  m_fieldDisplay = std::make_shared<FieldMapDisplay>(m_drivetrain, m_localizer, m_fieldMap);
  m_zoneFinder = std::make_shared<ZoneFinder>(m_localizer);
  m_targetFinder = std::make_shared<TargetFinder>(m_localizer, m_zoneFinder);

  m_climber = std::make_shared<Climber>();

  m_shooterTable = std::make_shared<ShooterTable>();
 std::cerr << "\tShooter table created..." << std::endl;
  m_intake = std::make_shared<Intake>();
   std::cerr << "\tIntake created..." << std::endl;
  m_collector = std::make_shared<Collector>();
   std::cerr << "\tCollector created..." << std::endl;
  m_spindexer = std::make_shared<Spindexer>();
   std::cerr << "\tSpindexer created..." << std::endl;
  m_kicker = std::make_shared<Kicker>();
   std::cerr << "\tKicker created..." << std::endl;
  m_shooterHood = std::make_shared<ShooterHood>();
   std::cerr << "\tShooterHood created..." << std::endl;
  m_flywheel = std::make_shared<Flywheel>();
   std::cerr << "\tFlywheel created..." << std::endl;
  //m_laser = std::make_shared<LaserCan>();

  m_autoRunner = std::make_shared<AutoRunner>(m_drivetrain, m_tagFinder, m_localizer, m_kicker, m_climber, m_flywheel, m_shooterHood, m_spindexer, m_turret, m_collector, m_intake, m_laser, m_shooterTable, m_targetFinder);

  std::cerr << "Mechanisms created..." << std::endl;

  // Assign default commands here after all subssytems are created to avoid using
  // uninitialized subsystems in default commands.
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_localizer).ToPtr());
  m_intake->SetDefaultCommand(IntakeTeleop(m_intake, m_OI, m_zoneFinder).ToPtr());
  m_collector->SetDefaultCommand(CollectorTeleop(m_collector, m_OI, m_drivetrain).ToPtr());
  m_spindexer->SetDefaultCommand(SpindexerTeleop(m_spindexer, m_OI).ToPtr());
  m_kicker->SetDefaultCommand(KickerTeleop(m_kicker, m_OI).ToPtr());
  m_shooterHood->SetDefaultCommand(HoodTeleop(m_shooterHood, m_OI, m_targetFinder, m_shooterTable, m_zoneFinder).ToPtr());
  m_flywheel->SetDefaultCommand(FlywheelTeleop(m_flywheel,m_OI, m_targetFinder, m_shooterTable).ToPtr());
  m_turret->SetDefaultCommand(TurretTeleop(m_turret, m_OI, m_targetFinder).ToPtr());
  m_climber->SetDefaultCommand(ClimberTeleop(m_climber, m_OI, m_zoneFinder).ToPtr());


  std::cerr << "\tDefault commands assigned..." << std::endl;

  // Autonomous Chooser:

  m_levelChooser.SetDefaultOption("No Level", noLevelAuto);
  m_levelChooser.AddOption("Basic Shot Auto", basicShotAuto);
  m_levelChooser.AddOption("Neutral Right Trench", neutralRightTrench);
  m_levelChooser.AddOption("Neutral Left Trench", neutralLeftTrench);
  m_levelChooser.AddOption("Half Neutral Right", halfNeutralRight);
  m_levelChooser.AddOption("Half Neutral Left", halfNeutralLeft);
  m_levelChooser.AddOption("Hub Auto", hubAuto);
  m_levelChooser.AddOption("Double Neutral Right", doubleNeutralRight);
  m_levelChooser.AddOption("Corner Shot Auto", cornerShotAuto);
  m_levelChooser.AddOption("Split Path Auto", splitAuto);

  frc::SmartDashboard::PutData("Level Chooser", &m_levelChooser);

  // Configure the button bindings
  ConfigureBindings();
  std::cerr << "Controller bindings configured..." << std::endl;
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  try {
    if(m_levelChooser.GetSelected() == weekZeroAuto) {
      return WeekZeroAuto::Create(m_spindexer, m_kicker, m_flywheel, m_shooterHood, m_turret);
    }
    else if (m_levelChooser.GetSelected() == basicAuto){
      return BasicAuto::Create(m_drivetrain, m_localizer);
    }
    else if (m_levelChooser.GetSelected() == basicShotAuto) {
      return Autos::BasicAutoShot(m_spindexer, m_kicker, m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable);
    }
    else if (m_levelChooser.GetSelected() == hubAuto) {
      return Autos::HubAuto(m_spindexer, m_kicker, m_turret, m_flywheel, m_shooterHood);
    }
    else if (
      m_levelChooser.GetSelected() == exampleAuto ||
      m_levelChooser.GetSelected() == neutralRightTrench ||
      m_levelChooser.GetSelected() == neutralLeftTrench ||
      m_levelChooser.GetSelected() == halfNeutralRight ||
      m_levelChooser.GetSelected() == halfNeutralLeft ||
      m_levelChooser.GetSelected() == doubleNeutralRight ||
      m_levelChooser.GetSelected() == cornerShotAuto ||
      m_levelChooser.GetSelected() == splitAuto
    ) {
      trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(m_levelChooser.GetSelected());
      return m_autoRunner->Create(trajectory);
    }
  }
  catch (...) {
    std::cerr << "Get Autonomous Command Threw Exception" << std::endl;
    return frc2::cmd::Idle(); // A do-nothing command from the commands factory.
  }

  return frc2::cmd::Idle(); // Do nothing.
}

// Called from Robot
void RobotContainer::DisabledInit() {

}

bool RobotContainer::DisabledPeriodic() {
  return false; // TODO: Fix return value.
}

void RobotContainer::TeleopInit() {

  // If the turret has not yet seen zero, zero it now.
  if (!m_turret->GetFeedback().haveZero) {
     frc2::CommandScheduler::GetInstance().Schedule(ZeroTurret(m_turret).ToPtr());
  }

   // TODO: Consider moving this back to Configuire Bindings.
   // Moved here to de-conflict DPAD in test mode.
  _operatorController.POVLeft().OnTrue(ZeroIntake(m_intake).ToPtr());
  _operatorController.POVUp().OnTrue(ZeroTurret(m_turret).ToPtr());
  _operatorController.POVRight().OnTrue(ZeroHood(m_shooterHood).ToPtr());
  _operatorController.POVDown().OnTrue(ZeroClimber(m_climber).ToPtr());

}


void RobotContainer::ConfigureBindings() {

  // Command bindings moved to TeleopInit().

}

void RobotContainer::TestInit() {

  std::cerr << "***** TestInit ****" << std::endl;
  
  // In test mode we run these manually:
  m_flywheel->RemoveDefaultCommand();
  m_shooterHood->RemoveDefaultCommand();

  // Launch some commands for test mode:
  frc2::CommandScheduler::GetInstance().Schedule(TestFlywheel(m_flywheel, m_OI).ToPtr());
  frc2::CommandScheduler::GetInstance().Schedule(TestHood(m_shooterHood, m_OI).ToPtr());


}

void RobotContainer::SetHubAcive(bool active) {
  m_OI->SetHubActive(active);
}