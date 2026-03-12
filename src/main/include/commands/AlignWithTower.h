#pragma once

#include <subsystems/DriveTrain.h>
#include <subsystems/AprilTagFinder.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <iostream>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/math.h>
#include <units/constants.h>

class AlignWithTower
    : public frc2::CommandHelper<frc2::Command, AlignWithTower> {
public: 
    AlignWithTower(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<AprilTagFinder> finder);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    private:
        std::shared_ptr<Drivetrain> m_drivetrain;
        std::shared_ptr<AprilTagFinder> m_finder;

        frc::Transform2d offset;
        frc::Pose2d targetPose;
        frc::Pose2d currentPose;
        frc::Pose2d deltaPose;
        
        frc::PIDController xController;
        frc::PIDController yController;
        frc::PIDController rotationController;

        bool hasTarget;
        int timesMissed;

};