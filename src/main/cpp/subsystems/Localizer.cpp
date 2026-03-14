#include "subsystems/Localizer.h"


Localizer::Localizer(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<AprilTagFinder> finder) : 
    _driveTrain(driveTrain),
    _finder(finder),
    _kinematics(_driveTrain->GetKinematics()),
    _estimator(std::make_shared<frc::SwerveDrivePoseEstimator<4U>>(_kinematics, driveTrain->GetOdometry().Rotation(), _driveTrain->GetSwerveModulePositions(), frc::Pose2d())),
    _lastUpdateTime(frc::Timer::GetFPGATimestamp())
{
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
    counter++;
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
    _speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, _estimator->GetEstimatedPosition().Rotation());
    
    if (counter == 50) {
        counter = 0;
        frc::SmartDashboard::PutNumber("Localize Measurements per second", (measurementCounter));
        measurementCounter = 0;
    }
    // Update localized output for debug:
    frc::SmartDashboard::PutNumber("Localizer/Pose(x)", _pose.X().value());
    frc::SmartDashboard::PutNumber("Localizer/Pose(y)", _pose.Y().value());
    frc::SmartDashboard::PutNumber("Localizer/Pose(q)", _pose.Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("Localizer/Vel(x)", _speeds.vx.value());
    frc::SmartDashboard::PutNumber("Localizer/Vel(y)", _speeds.vy.value());
    frc::SmartDashboard::PutNumber("Localizer/Vel(q)", _speeds.omega.value());
}

frc::ChassisSpeeds Localizer::getSpeeds() {
    return _speeds;
}

bool Localizer::measurementStable(){
        units::meters_per_second_t linearSpeed = units::math::sqrt((_driveTrain->GetChassisSpeeds().vx)*(_driveTrain->GetChassisSpeeds().vx) + (_driveTrain->GetChassisSpeeds().vy)*(_driveTrain->GetChassisSpeeds().vy));
        units::radians_per_second_t angularSpeed = units::math::abs(_driveTrain->GetChassisSpeeds().omega);
        return (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold);
}