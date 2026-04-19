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

// const std::string RobotContainer::basicAuto = "Basic Auto";
// const std::string RobotContainer::rightTrenchFull = "RightTrenchFull";
// const std::string RobotContainer::leftTrenchFull = "LeftTrenchFull";
// const std::string RobotContainer::rightTrenchHalf = "RightTrenchHalf";
// const std::string RobotContainer::leftTrenchHalf = "LeftTrenchHalf";

const std::string RobotContainer::noLevelAuto = "No Auto";
const std::string RobotContainer::startLine = "Start Line";
const std::string RobotContainer::centerHub = "CenterHub";
const std::string RobotContainer::centerDepotOutpost = "CenterDepotOutpost";
const std::string RobotContainer::centerDepotOutpostClimb = "CenterDepotOutpostClimb";
const std::string RobotContainer::rightBumpSteal = "RightBumpSteal";
const std::string RobotContainer::rightTrenchHalfOutpost = "RightTrenchHalfOutpost";
const std::string RobotContainer::rightTrenchHalfDouble = "RightTrenchHalfDouble";
const std::string RobotContainer::rightTrenchHalfDoubleBump = "RightTrenchHalfDoubleBump";
const std::string RobotContainer::leftTrenchHalfDouble = "LeftTrenchHalfDouble";
const std::string RobotContainer::leftTrenchHalfDoubleBump = "LeftTrenchHalfDoubleBump";
const std::string RobotContainer::leftBumpFull = "LeftBumpFull";

const std::string RobotContainer::basicTest = "BasicTest";

RobotContainer::RobotContainer() :
m_operatorController(1),
m_controlBindings(false),
m_startDelaySeconds(0.0)
{
  haveTraj = false;
  autoTraj = "none";

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
  m_ballisticShot = std::make_shared<BallisticShot>(m_targetFinder);

  m_targetFinder->SetBallisticShot(m_ballisticShot);

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
  m_bling = std::make_shared<Bling>();
   std::cerr << "\tBling created..." << std::endl;
  m_autoRunner = std::make_shared<AutoRunner>(m_drivetrain, m_tagFinder, m_localizer, m_kicker, m_climber, m_flywheel, m_shooterHood, m_spindexer, m_turret, m_collector, m_intake, m_laser, m_shooterTable, m_targetFinder, m_bling, m_ballisticShot);

  std::cerr << "Mechanisms created..." << std::endl;

  // Assign default commands here after all subssytems are created to avoid using
  // uninitialized subsystems in default commands.
  // m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_localizer).ToPtr());
  // m_intake->SetDefaultCommand(IntakeTeleop(m_intake, m_OI, m_zoneFinder).ToPtr());
  // m_collector->SetDefaultCommand(CollectorTeleop(m_collector, m_OI, m_drivetrain).ToPtr());
  // m_spindexer->SetDefaultCommand(SpindexerTeleop(m_spindexer, m_kicker, m_OI).ToPtr());
  // m_kicker->SetDefaultCommand(KickerTeleop(m_kicker, m_OI).ToPtr());
  // m_shooterHood->SetDefaultCommand(HoodTeleop(m_shooterHood, m_OI, m_targetFinder, m_shooterTable, m_zoneFinder, m_ballisticShot).ToPtr());
  // m_flywheel->SetDefaultCommand(FlywheelTeleop(m_flywheel,m_OI, m_targetFinder, m_shooterTable, m_ballisticShot).ToPtr());
  // m_turret->SetDefaultCommand(TurretTeleop(m_turret, m_OI, m_targetFinder, m_drivetrain).ToPtr());
  // m_climber->SetDefaultCommand(ClimberTeleop(m_climber, m_OI, m_zoneFinder).ToPtr());
  // m_bling->SetDefaultCommand(BlingTeleop(m_bling, m_OI).ToPtr());

  std::cerr << "\tDefault commands assigned..." << std::endl;

  // Autonomous Chooser:

  // m_levelChooser.AddOption("Right_Trench_Full", rightTrenchFull);
  // m_levelChooser.AddOption("Left_Trench_Full", leftTrenchFull);
  // m_levelChooser.AddOption("Right_Trench_Half", rightTrenchHalf);
  // m_levelChooser.AddOption("Left_Trench_Half", leftTrenchHalf);

  m_levelChooser.SetDefaultOption("No Level", noLevelAuto);
  m_levelChooser.AddOption("Start_Line", startLine);
  m_levelChooser.AddOption("Center_Hub", centerHub);
  m_levelChooser.AddOption("Center_Depot_Outpost", centerDepotOutpost);
  m_levelChooser.AddOption("Center_Depot_Oupost_Climb", centerDepotOutpostClimb);
  m_levelChooser.AddOption("Right_Bump_Steal", rightBumpSteal);
  m_levelChooser.AddOption("Right_Trench_Half_Outpost", rightTrenchHalfOutpost);
  m_levelChooser.AddOption("Right_Trench_Half_Double", rightTrenchHalfDouble);
  m_levelChooser.AddOption("Right_Trench_Half_Double_Bump", rightTrenchHalfDoubleBump);
  m_levelChooser.AddOption("Left_Trench_Half_Double", leftTrenchHalfDouble);
  m_levelChooser.AddOption("Left_Trench_Half_Double_Bump", leftTrenchHalfDoubleBump);
  m_levelChooser.AddOption("Outliers_Right", leftBumpFull);
  m_levelChooser.AddOption("Basic Test", basicTest);

  frc::SmartDashboard::PutData("Level Chooser", &m_levelChooser);

  frc::SmartDashboard::PutNumber("Start Delay(s)", m_startDelaySeconds);
  // Configure the button bindings
  ConfigureBindings();
  std::cerr << "Controller bindings configured..." << std::endl;
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  try {

    // Grab our delay in seconds:
    auto delay = units::time::second_t(frc::SmartDashboard::GetNumber("Start Delay", 0.0));
    std::cerr << "**Auto Start Delay(s): " << delay.value() << std::endl;

    if (m_levelChooser.GetSelected() == startLine) {
      return frc2::cmd::Sequence(frc2::cmd::Wait(delay), ZeroTurret(m_turret).ToPtr(), ZeroClimber(m_climber).ToPtr(), 
                            Autos::BasicAutoShot(m_spindexer, m_kicker, m_turret, m_flywheel, m_shooterHood, m_targetFinder, m_shooterTable, m_ballisticShot));
    }
    else if (m_levelChooser.GetSelected() == centerHub) {
      return frc2::cmd::Sequence(frc2::cmd::Wait(delay), ZeroTurret(m_turret).ToPtr(), ZeroClimber(m_climber).ToPtr(), 
                                Autos::HubAuto(m_spindexer, m_kicker, m_turret, m_flywheel, m_shooterHood));
    }

    else if (trajectory.has_value()) {
      
      bool putIntakeOut = true;

      if (m_levelChooser.GetSelected() == centerDepotOutpost) {
        putIntakeOut = false;

      } else if (m_levelChooser.GetSelected() == leftBumpFull) {
        putIntakeOut = false;

      } else if (m_levelChooser.GetSelected() == centerDepotOutpostClimb) {
        putIntakeOut = false;

      } else if (m_levelChooser.GetSelected() == rightBumpSteal) {
        putIntakeOut = false;

      } else if (m_levelChooser.GetSelected() == rightTrenchHalfDoubleBump) {
        putIntakeOut = false;
      } else if (m_levelChooser.GetSelected() == leftTrenchHalfDouble) {
        putIntakeOut = false;
      } else if (m_levelChooser.GetSelected() == leftTrenchHalfDoubleBump) {
        putIntakeOut = false;
      } else if (m_levelChooser.GetSelected() == rightTrenchHalfDouble) {
        putIntakeOut = false;
      } else if (m_levelChooser.GetSelected() == basicTest) {
        putIntakeOut = false;
      }

      frc::SmartDashboard::PutBoolean("Autos/Put Intake Out", putIntakeOut);
      frc::SmartDashboard::PutNumber("Autos/Start Auto", frc::Timer::GetFPGATimestamp().value());
      return m_autoRunner->Create(trajectory, delay, putIntakeOut);
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
  haveTraj = false;
}

bool RobotContainer::DisabledPeriodic() {
  return LoadTrajectory();
}

bool RobotContainer::LoadTrajectory() {
  if (
      m_levelChooser.GetSelected() == centerDepotOutpost ||
      m_levelChooser.GetSelected() == centerDepotOutpostClimb ||
      m_levelChooser.GetSelected() == rightBumpSteal ||
      m_levelChooser.GetSelected() == rightTrenchHalfOutpost ||
      m_levelChooser.GetSelected() == rightTrenchHalfDouble ||
      m_levelChooser.GetSelected() == rightTrenchHalfDoubleBump ||
      m_levelChooser.GetSelected() == leftTrenchHalfDouble ||
      m_levelChooser.GetSelected() == leftTrenchHalfDoubleBump ||
      m_levelChooser.GetSelected() == leftBumpFull ||
      m_levelChooser.GetSelected() == basicTest
  ) {
    autoTraj = m_levelChooser.GetSelected();
    frc::SmartDashboard::PutNumber("Autos/Grabed Choreo", frc::Timer::GetFPGATimestamp().value());
    trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(m_levelChooser.GetSelected());
  }
  return trajectory.has_value();
}

void RobotContainer::TeleopInit() {

  // uninitialized subsystems in default commands.
  m_drivetrain->SetDefaultCommand(TeleopDrive(m_drivetrain, m_OI, m_localizer).ToPtr());
  m_intake->SetDefaultCommand(IntakeTeleop(m_intake, m_OI, m_zoneFinder).ToPtr());
  m_collector->SetDefaultCommand(CollectorTeleop(m_collector, m_OI, m_drivetrain).ToPtr());
  m_spindexer->SetDefaultCommand(SpindexerTeleop(m_spindexer, m_kicker, m_OI).ToPtr());
  m_kicker->SetDefaultCommand(KickerTeleop(m_kicker, m_OI).ToPtr());
  m_shooterHood->SetDefaultCommand(HoodTeleop(m_shooterHood, m_OI, m_targetFinder, m_shooterTable, m_zoneFinder, m_ballisticShot).ToPtr());
  m_flywheel->SetDefaultCommand(FlywheelTeleop(m_flywheel,m_OI, m_targetFinder, m_shooterTable, m_ballisticShot).ToPtr());
  m_turret->SetDefaultCommand(TurretTeleop(m_turret, m_OI, m_targetFinder, m_drivetrain).ToPtr());
  m_climber->SetDefaultCommand(ClimberTeleop(m_climber, m_OI, m_zoneFinder).ToPtr());
  m_bling->SetDefaultCommand(BlingTeleop(m_bling, m_OI).ToPtr());

  // If the turret has not yet seen zero, zero it now.
  if (!m_turret->GetFeedback().haveZero) {
     frc2::CommandScheduler::GetInstance().Schedule(ZeroTurret(m_turret).ToPtr());
  }

   // TODO: Consider moving this back to Configuire Bindings.
   // Moved here to de-conflict DPAD in test mode.
  if (!m_controlBindings) {
    m_operatorController.POVLeft().OnTrue(ZeroIntake(m_intake).ToPtr());
    m_operatorController.POVUp().OnTrue(ZeroTurret(m_turret).ToPtr());
    m_operatorController.POVRight().OnTrue(ZeroHood(m_shooterHood).ToPtr());
    m_operatorController.POVDown().OnTrue(ZeroClimber(m_climber).ToPtr());
    m_controlBindings = true;
  }

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