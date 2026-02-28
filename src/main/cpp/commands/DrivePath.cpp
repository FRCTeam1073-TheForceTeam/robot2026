// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DrivePath.h"

DrivePath::DrivePath(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory) :
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
  AddRequirements({m_drivetrain.get(), m_localizer.get()});
}

// Called when the command is initially scheduled.
void DrivePath::Initialize() {
  frc::SmartDashboard::PutBoolean("Has Trajectory", trajectory.has_value());

  startTime = frc::Timer::GetFPGATimestamp();
  if(trajectory.has_value()) {
    endTime = trajectory.value().GetTotalTime();
    robotPose = m_localizer->getPose();
    std::optional<frc::Pose2d> initPose = trajectory.value().GetInitialPose();
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
  currentTime = frc::Timer::GetFPGATimestamp() - startTime;
  frc::SmartDashboard::PutNumber("DrivePath/CurrentTime", currentTime.value());
  frc::SmartDashboard::PutBoolean("DrivePath/Trajectory", trajectory.has_value());

  if(trajectory.has_value()) {
    const auto &traj = trajectory.value();

    currentSample = traj.SampleAt(currentTime);

    frc::SmartDashboard::PutBoolean("DrivePath/Current Sample", currentSample.has_value());

    if(currentSample.has_value()) {
      const auto &cur = currentSample.value();
      frc::ChassisSpeeds sample_speed = cur.GetChassisSpeeds();
      maxVelocity = 1_mps * std::sqrt(std::pow(sample_speed.vx.value(), 2) + std::sqrt(std::pow(sample_speed.vy.value(), 2))); 
      maxAngularVelocity = std::sqrt(std::pow(sample_speed.vx.value(), 2) + std::sqrt(std::pow(sample_speed.vy.value(), 2))) * 2_rad_per_s * std::numbers::pi;  //TODO: Find how to access choreo values

      xVelocity = xController.Calculate(robotPose.X().value(), sample_speed.vx.value()) * 1_mps;
      yVelocity = yController.Calculate(robotPose.Y().value(), sample_speed.vy.value()) * 1_mps; 
      thetaVelocity = thetaController.Calculate(robotPose.Rotation().Radians().value(), cur.GetPose().Rotation().Radians().value()) * 1_rad_per_s;

      xVelocity = std::clamp(xVelocity, -maxVelocity, maxVelocity);
      yVelocity = std::clamp(yVelocity, -maxVelocity, maxVelocity);
      thetaVelocity = units::angular_velocity::radians_per_second_t(std::clamp(thetaVelocity.value(), -maxAngularVelocity.value(), maxAngularVelocity.value()));

      frc::SmartDashboard::PutNumber("DrivePath/TargetX", sample_speed.vx.value());
      frc::SmartDashboard::PutNumber("DrivePath/TargetY", sample_speed.vy.value());
      frc::SmartDashboard::PutNumber("DrivePath/TargetTheta", cur.GetPose().Rotation().Radians().value());

      frc::SmartDashboard::PutNumber("DrivePath/MaxVelocity", maxVelocity.value());

      frc::SmartDashboard::PutNumber("DrivePath/CommandedVx", xVelocity.value());
      frc::SmartDashboard::PutNumber("DrivePath/CommandedVy", yVelocity.value());
      frc::SmartDashboard::PutNumber("DrivePath/CommandedVw", thetaVelocity.value());
      
      // m_drivetrain->SetChassisSpeeds(
      //   frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      //     xVelocity,
      //     yVelocity,
      //     thetaVelocity,
      //     m_localizer->getPose().Rotation()
      //   )
      // );

      //TODO: test this
      m_drivetrain->SetChassisSpeeds(
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          sample_speed.vx,
          sample_speed.vy,
          sample_speed.omega,
          m_localizer->getPose().Rotation()
        )
      );
    }
    else {
      std::cerr << "DrivePath No Sample Found" << std::endl;
      m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
    }
  }
  else {
    frc::SmartDashboard::PutString("DrivePath/Status", "No Trajectory Found");
    std::cerr << "DrivePath No Trajectory Found" << std::endl;
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
