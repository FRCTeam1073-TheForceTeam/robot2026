#include <commands/AlignWithTower.h>
AlignWithTower::AlignWithTower(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<AprilTagFinder> finder) : m_drivetrain(drivetrain),
m_finder(finder),
xController(frc::PIDController(2, 0, 0.03)),
yController(frc::PIDController(2, 0, 0.03)),
rotationController(frc::PIDController(1.875, 0, 0.03)),
hasTarget(false),
timesMissed(0)
{
    AddRequirements({m_drivetrain.get()});
}
void AlignWithTower::Initialize() {
    offset = frc::Transform2d(frc::Translation2d(1.5_m,-1.143_m),frc::Rotation2d(0.5_rad*units::constants::pi));
    // frc::SmartDashboard::PutNumber("AlignWithTower/targetX", targetPose.X().value());
    // frc::SmartDashboard::PutNumber("AlignWithTower/targetY", targetPose.Y().value());
    // frc::SmartDashboard::PutNumber("AlignWithTower/targetAngle", targetPose.Rotation().Radians().value());
    xController.Reset();
    yController.Reset();
    rotationController.Reset();
    currentPose = frc::Pose2d();
    timesMissed = 0;
    rotationController.EnableContinuousInput(-units::constants::pi,units::constants::pi);
}

void AlignWithTower::Execute() {
    auto april_tag_pose = m_finder->getPoseToTag(16);
    currentPose = m_drivetrain->GetOdometry();
    if(april_tag_pose.has_value())
    {
        targetPose = currentPose+april_tag_pose.value()+offset;
        hasTarget = true;
        timesMissed = 0;
    }
    else{
        timesMissed++;
    }
    if(!hasTarget) return;

    deltaPose = currentPose.RelativeTo(targetPose);
    
    frc::SmartDashboard::PutNumber("AlignWithTower/currentX", currentPose.X().value());
    frc::SmartDashboard::PutNumber("AlignWithTower/currentY", currentPose.Y().value());
    frc::SmartDashboard::PutNumber("AlignWithTower/currentAngle", currentPose.Rotation().Radians().value());
    auto xVel = units::velocity::meters_per_second_t(xController.Calculate(currentPose.X().value(),targetPose.X().value()));
    auto yVel = units::velocity::meters_per_second_t(yController.Calculate(currentPose.Y().value(),targetPose.Y().value()));
    auto wVel = units::angular_velocity::radians_per_second_t(rotationController.Calculate(currentPose.Rotation().Radians().value(),targetPose.Rotation().Radians().value()));
    frc::SmartDashboard::PutNumber("AlignWithTower/velocityX", xVel.value());
    frc::SmartDashboard::PutNumber("AlignWithTower/velocityY", yVel.value());
    frc::SmartDashboard::PutNumber("AlignWithTower/velocityAngle", wVel.value());
    m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds(xVel,yVel,wVel));
}

void AlignWithTower::End(bool interrupted) {
    m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
}

bool AlignWithTower::IsFinished() {
    if(timesMissed>60||(deltaPose.Translation().Norm()<0.01_m && units::math::abs(deltaPose.Rotation().Radians()) < 0.01_rad))
        return true;
    return false;
}