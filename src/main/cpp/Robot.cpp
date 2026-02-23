// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//    !!ATTENTION!!     Most of this code is commented out because it was copied directly from weewee 2026 cpp code and 
//                      might not work. Create working code on the designated branches, and do not un-comment on main until
//                      the branch throws no errors

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "grpl/CanBridge.h"

Robot::Robot() {
  // Required to support laser-can debugging and configuration.
  grpl::start_can_bridge();

  // Scheduler instance debugging:
  // frc::SmartDashboard::PutData("Scheduler", frc2::CommandScheduler::GetInstance());

    // TODO: Integrate CppTrace library to speed up debugging
  try {
    m_container = std::make_unique<RobotContainer>(); // Actually create robot container here so we can capture errors for debugging.
  } catch(std::exception& e) {
    std::cerr << "CREATION OF ROBOT CONTAINER THREW AN EXCEPTION!: " << e.what() << std::endl;
  } catch(...) {
    std::cerr << "UNKONWN EXCEPTION CREATING ROBOT CONTAINER!" << std::endl;
  }

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  // TODO: Integrate CppTrace library to speed up debugging
  try {
    frc2::CommandScheduler::GetInstance().Run();
  } catch (std::exception& e) {
    std::cerr << "SCHEDULER RUN THREW EXCEPTION!: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "SCHEDULER RUN THREW UNKNOWN EXCEPTION!" << std::endl;
  }
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  std::cerr << "Disabled Init..." << std::endl;
    // TODO: Integrate CppTrace library to speed up debugging
  try {
    // Delegate to container function:
    m_container->DisabledInit();
  } catch (std::exception& e) {
    std::cerr << "CONTAINER DISABLEDINIT THREW EXCEPTION!: " << e.what() << std::endl;
  }
}

void Robot::DisabledPeriodic() {
    // TODO: Integrate CppTrace library to speed up debugging
  try {
    // Delegate to container function:
    m_container->DisabledPeriodic();
  } catch (std::exception& e) {
    std::cerr << "CONTAINER DISABLEDPERIODIC THREW EXCEPTION!: " << e.what() << std::endl;
  }
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  std::cerr << "Autonomous Init..." << std::endl;

  try {
    m_autonomousCommand = m_container->GetAutonomousCommand();

    if (m_autonomousCommand) {
      frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value().get());
    } else {
      std::cerr << "UNEXPLAINED MISSING AUTONOMOUS COMMAND!" << std::endl;
    }

  } catch (std::exception& e) { 
    std::cerr << "AUTONOMOUS INIT THREW EXCEPTION!: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "AUTONOMOUS INIT THREW UNKNOWN EXCEPTION!" << std::endl;
  }
}

// TODO:

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  std::cerr << "TeleopInit..." << std::endl;
  
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand.value().Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {

}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {

}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
