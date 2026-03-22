// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DrivePath.h"

DrivePath::DrivePath(std::shared_ptr<Drivetrain>& drivetrain, std::shared_ptr<Localizer>& localizer, 
                    std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) :
  m_drivetrain(drivetrain),
  m_localizer(localizer),
  trajectory(trajectory),
  xController{4.8, 0, 0.01},
  yController{4.8, 0, 0.01},
  thetaController{1.5, 0.0, 0.01}
{
  quit = false;
  thetaController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  frc::SmartDashboard::PutString("DrivePath/Status", "Idle");
  AddRequirements(m_drivetrain.get());
}

// Called when the command is initially scheduled.
void DrivePath::Initialize() {
  frc::SmartDashboard::PutBoolean("Has Trajectory", trajectory.has_value());

  startTime = frc::Timer::GetFPGATimestamp();

  if(trajectory.has_value()) {
    //get the initial time and pose of the robot
    endTime = trajectory.value().GetTotalTime();
    robotPose = m_localizer->getPose();

    //if the current position is far from the start position quit
    std::optional<frc::Pose2d> initPose = trajectory.value().GetInitialPose(IsRedAlliance());
    if (initPose.has_value()){
      frc::Transform2d diff = (robotPose - initPose.value());
      if(diff.Translation().Norm() >= 2_m) {
        quit = true;
      }
    }
  }

  currentTime = 0.01_s;

  xController.Reset();
  yController.Reset();
  thetaController.Reset();
}

// Called repeatedly when this Command is scheduled to run
void DrivePath::Execute() {
  //get the start time and position
  currentTime = frc::Timer::GetFPGATimestamp() - startTime;
  robotPose = m_localizer->getPose();

  frc::SmartDashboard::PutNumber("DrivePath/CurrentTime", currentTime.value());
  frc::SmartDashboard::PutBoolean("DrivePath/Trajectory", trajectory.has_value());

  if (trajectory.has_value()) {
    const auto &traj = trajectory.value();

    //fetch current sample
    currentSample = traj.SampleAt(currentTime, IsRedAlliance());

    frc::SmartDashboard::PutBoolean("DrivePath/Current Sample", currentSample.has_value());

    if(currentSample.has_value()) {
      const auto &traj_sample = currentSample.value();
      frc::ChassisSpeeds trajectory_speeds = traj_sample.GetChassisSpeeds();
      maxVelocity = 2.5_mps; 
      maxAngularVelocity = 2.5_rad_per_s;

      //v = PID(Tranfrom + Robot_Pose) + Forward_Velocity * alpha
      //velocity = feedback + feedforward
      xVelocity = xController.Calculate(robotPose.X().value(), traj_sample.x.value()) * 1_mps + (trajectory_speeds.vx * 0.6);
      yVelocity = yController.Calculate(robotPose.Y().value(), traj_sample.y.value()) * 1_mps + (trajectory_speeds.vy * 0.6); 
      thetaVelocity = thetaController.Calculate(robotPose.Rotation().Radians().value(), traj_sample.heading.value()) * 1_rad_per_s + (trajectory_speeds.omega * 0.6);

      xVelocity = std::clamp(xVelocity, -maxVelocity, maxVelocity);
      yVelocity = std::clamp(yVelocity, -maxVelocity, maxVelocity);
      thetaVelocity = units::angular_velocity::radians_per_second_t(std::clamp(thetaVelocity.value(), -maxAngularVelocity.value(), maxAngularVelocity.value()));

      frc::SmartDashboard::PutNumber("DrivePath/TargetX", trajectory_speeds.vx.value());
      frc::SmartDashboard::PutNumber("DrivePath/TargetY", trajectory_speeds.vy.value());
      frc::SmartDashboard::PutNumber("DrivePath/TargetTheta", traj_sample.GetPose().Rotation().Radians().value());

      frc::SmartDashboard::PutNumber("DrivePath/MaxVelocity", maxVelocity.value());

      frc::SmartDashboard::PutNumber("DrivePath/CommandedVx", xVelocity.value());
      frc::SmartDashboard::PutNumber("DrivePath/CommandedVy", yVelocity.value());
      frc::SmartDashboard::PutNumber("DrivePath/CommandedVw", thetaVelocity.value());
      
      //set a field relative chassis speed
      m_drivetrain->SetChassisSpeeds(
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          xVelocity,
          yVelocity,
          thetaVelocity,
          m_localizer->getPose().Rotation()
        )
      );
    }
    else {
      std::cerr << "DrivePath No Sample Found" << std::endl;
      m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
      quit = true;
    }
  }
  else {
    frc::SmartDashboard::PutString("DrivePath/Status", "No Trajectory Found");
    std::cerr << "DrivePath No Trajectory Found" << std::endl;
    quit = true;
  }
}

// Called once the command ends or is interrupted.
void DrivePath::End(bool interrupted) {
  frc::SmartDashboard::PutBoolean("DrivePath/End", true);
  m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
}

// Returns true when the command should end.
bool DrivePath::IsFinished() {
  frc::SmartDashboard::PutBoolean("DrivePath/Past Time", currentTime >= endTime);
  frc::SmartDashboard::PutBoolean("DrivePath/Quit", quit);
  if(currentTime >= endTime || quit) {
    frc::SmartDashboard::PutString("DrivePath/Status", "Finished");
    std::cout << "IsFinishedRun" << std::endl;
    return true;
  }
  return false;
}

bool DrivePath::IsRedAlliance() {
  auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue);
  return alliance == frc::DriverStation::kRed;
}