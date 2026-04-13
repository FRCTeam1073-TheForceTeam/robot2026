#pragma once

#include <frc2/command/SubsystemBase.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/AprilTagFinder.h"
#include "subsystems/FieldMap.h"

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Transform3d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <units/length.h>
#include <cmath> 
#include <wpi/SymbolExports.h>
#include <wpi/array.h>
#include <frc/Timer.h>
#include <chrono>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/estimator/PoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "units/time.h"

class Localizer : public frc2::SubsystemBase {
    public:

    // Between 0.0 and 1.0:  1.0 is unfiltered, 0.0 is no updates.
    static constexpr double VelocityFilterAlpha = 0.7;

    Localizer(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<AprilTagFinder> finder);
    
    void InitSendable(wpi::SendableBuilder &builder) override;

    units::time::millisecond_t getTimeGap() { return timeGap; }

    void setTimeGap(units::time::millisecond_t time) { timeGap = time; }

    // creates an entirely new estimator so the rotation is reset for sure
    void resetPose(frc::Pose2d newPos);

    void resetOrientation();

    units::velocity::meters_per_second_t getLinearSpeedThreshold() { return linearSpeedThreshold; }

    void setLinearSpeedThreshold(units::velocity::meters_per_second_t speed) { linearSpeedThreshold = speed; }

    units::angular_velocity::radians_per_second_t getAngularSpeedThreshold() { return angularSpeedThreshold; }

    void setAngularSpeedThreshold(units::angular_velocity::radians_per_second_t angularSpeed) { angularSpeedThreshold = angularSpeed; }

    void Periodic() override;

    // Returns field-centric, localizer based position estimate.
    frc::Pose2d getPose() { return _pose; }

    // Returns field-centric, localizer based speeds.
    frc::ChassisSpeeds getSpeeds();

    void additionalSensorMeasurement(int id, FieldMap fieldMap);

    bool measurementStable();

    private:

    std::shared_ptr<Drivetrain> _driveTrain;  
    std::shared_ptr<AprilTagFinder> _finder;
    frc::SwerveDriveKinematics<4U> _kinematics;
    std::shared_ptr<frc::SwerveDrivePoseEstimator<4U>> _estimator;
    frc::ChassisSpeeds _speeds; // Cached field centric velocity.
    frc::Pose2d _pose; // Cached localized pose.

    units::time::second_t _lastUpdateTime;
    int measurementCounter;
    int counter;
    units::time::millisecond_t timeGap{30};
    units::velocity::meters_per_second_t linearSpeedThreshold{2.5};
    units::angular_velocity::radians_per_second_t angularSpeedThreshold{2};
};