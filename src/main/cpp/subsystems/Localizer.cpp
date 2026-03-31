#include "subsystems/Localizer.h"


Localizer::Localizer(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<AprilTagFinder> finder) : 
    _driveTrain(driveTrain),
    _finder(finder),
    _kinematics(_driveTrain->GetKinematics()),
    _estimator(std::make_shared<frc::SwerveDrivePoseEstimator<4U>>(_kinematics, driveTrain->GetOdometry().Rotation(), _driveTrain->GetSwerveModulePositions(), frc::Pose2d())),
    _lastUpdateTime(frc::Timer::GetFPGATimestamp())
{
    counter = 0;
    measurementCounter = 0;
    _speeds.vx = 0.0_mps;
    _speeds.vy = 0.0_mps;
    _speeds.omega = 0.0_rad_per_s;
}

void Localizer::InitSendable(wpi::SendableBuilder &builder) {
    
    builder.SetSmartDashboardType("Localizer");
}

void Localizer::resetPose(frc::Pose2d newPos) {
    _estimator = std::make_shared<frc::SwerveDrivePoseEstimator<4U>>(_kinematics, _driveTrain->GetGyroHeading(), _driveTrain->GetSwerveModulePositions(), newPos);
}

void Localizer::Periodic() {
    units::time::second_t now = frc::Timer::GetFPGATimestamp();	
    _estimator->UpdateWithTime(now, _driveTrain->GetGyroHeading(), _driveTrain->GetSwerveModulePositions());

    if (now - _lastUpdateTime > timeGap && measurementStable()) {
        std::vector<AprilTagFinder::VisionMeasurement> measurements = _finder->getAllMeasurements();
        for (uint index = 0; index < measurements.size(); index++){
            AprilTagFinder::VisionMeasurement current_measurement = measurements[index];

            _estimator->AddVisionMeasurement(current_measurement._pose, current_measurement._timeStamp, current_measurement._stddevs);
            measurementCounter++;
        }
        _lastUpdateTime = now;
        
        _finder->clearMeasurements();
    }

    // Cache output:
    _pose = _estimator->GetEstimatedPosition();
    // Compute speeds in fleid coordinates:
    auto speeds = _driveTrain->GetChassisSpeeds();
    auto field_speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, _estimator->GetEstimatedPosition().Rotation());

    // Simplistic IIR update of reported field-centric speeds:
    _speeds.vx = (1.0 - VelocityFilterAlpha) * _speeds.vx + VelocityFilterAlpha * field_speeds.vx;
    _speeds.vy = (1.0 - VelocityFilterAlpha) * _speeds.vy + VelocityFilterAlpha * field_speeds.vy;
    _speeds.omega = (1.0 - VelocityFilterAlpha) * _speeds.omega + VelocityFilterAlpha * field_speeds.omega;
    
    if (counter >= 50) {
        frc::SmartDashboard::PutNumber("Localizer/PS", measurementCounter);
        measurementCounter = 0;
        counter = 0;
    }
    else {
        counter = counter + 1;
    }
    // Update localized output for debug:
    frc::SmartDashboard::PutNumber("Localizer/Pose(x)", _pose.X().value());
    frc::SmartDashboard::PutNumber("Localizer/Pose(y)", _pose.Y().value());
    frc::SmartDashboard::PutNumber("Localizer/Pose(q)", _pose.Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("Localizer/Vel(x)", _speeds.vx.value());
    frc::SmartDashboard::PutNumber("Localizer/Vel(y)", _speeds.vy.value());
    frc::SmartDashboard::PutNumber("Localizer/Vel(q)", _speeds.omega.value());
    frc::SmartDashboard::PutNumber("Localizer/MC", measurementCounter);

}

frc::ChassisSpeeds Localizer::getSpeeds() {
    return _speeds;
}

bool Localizer::measurementStable(){
        units::meters_per_second_t linearSpeed = units::math::sqrt((_driveTrain->GetChassisSpeeds().vx)*(_driveTrain->GetChassisSpeeds().vx) + (_driveTrain->GetChassisSpeeds().vy)*(_driveTrain->GetChassisSpeeds().vy));
        units::radians_per_second_t angularSpeed = units::math::abs(_driveTrain->GetChassisSpeeds().omega);
        return (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold);
}