#include <commands/AlignWithTower.h>
AlignWithTower::AlignWithTower(std::shared_ptr<Drivetrain> drivetrain) : m_drivetrain(drivetrain),
xController(frc::PIDController(2, 0, 0.03)),
yController(frc::PIDController(2, 0, 0.03)),
rotationController(frc::PIDController(1.875, 0, 0.03))
{
    AddRequirements({m_drivetrain.get()});
}
void AlignWithTower::Initialize() {
    rotationController.EnableContinuousInput(-units::constants::pi,units::constants::pi);
}

void AlignWithTower::Execute() {
    delta_pose = current_pose.RelativeTo(target_pose);
    auto xVel = units::velocity::meters_per_second_t(xController.Calculate(current_pose.X().value(),target_pose.X().value()));
    auto yVel = units::velocity::meters_per_second_t(yController.Calculate(current_pose.Y().value(),target_pose.Y().value()));
    auto wVel = units::angular_velocity::radians_per_second_t(rotationController.Calculate(current_pose.Rotation().Radians().value(),target_pose.Rotation().Radians().value()));
    m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds(xVel,yVel,wVel));
}

void AlignWithTower::End(bool interrupted) {
    m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
}

bool AlignWithTower::IsFinished() {
    if(delta_pose.Translation().Norm()<0.01_m && units::math::abs(delta_pose.Rotation().Radians()) < 0.01_rad)
        return true;
    return false;
}