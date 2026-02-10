#include "subsystems/Localizer.h"



Localizer::Localizer(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<AprilTagFinder> finder) : 
    _driveTrain(driveTrain),
    _finder(finder),
    _kinematics(_driveTrain->GetKinematics()),
    _estimator(std::make_shared<frc::SwerveDrivePoseEstimator<4U>>(_kinematics, driveTrain->GetOdometry().Rotation(), _driveTrain->GetSwerveModulePositions(), frc::Pose2d())),
    _lastUpdateTime(frc::Timer::GetFPGATimestamp())
{}

void Localizer::InitSendable(wpi::SendableBuilder &builder) {
    
    builder.SetSmartDashboardType("Localizer");
    builder.AddDoubleProperty("Time between updates", [this] { return getTime().value(); },   [this] (double timeGap) { setTime(units::time::millisecond_t(timeGap)); });
    builder.AddDoubleProperty("Linear Speed Thres", [this] { return getLinearSpeed().value(); }, [this] (double linearSpeed) { setTime(units::time::millisecond_t(timeGap));} );
    builder.AddDoubleProperty("Angular Speed Thres", [this] { return getAngularSpeed().value(); }, [this] (double angularSpeed) { setTime(units::time::millisecond_t(timeGap));} );

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
        frc::SmartDashboard::PutNumber("Localize Measurements", measurementCounter);
    }
}

bool Localizer::measurementStable(){
        units::meters_per_second_t linearSpeed = units::math::sqrt((_driveTrain->GetChassisSpeeds().vx)*(_driveTrain->GetChassisSpeeds().vx) + (_driveTrain->GetChassisSpeeds().vy)*(_driveTrain->GetChassisSpeeds().vy));
        units::radians_per_second_t angularSpeed = units::math::abs(_driveTrain->GetChassisSpeeds().omega);
        return (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold);
}