// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "subsystems/FieldMap.h"
#include <frc2/command/SubsystemBase.h>
#include <vector>

#include "subsystems/Turret.h"
#include "subsystems/DriveTrain.h"
//Photon Vision Includes
#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <units/length.h>
#include <units/angle.h>


class AprilTagFinder : public frc2::SubsystemBase {
    public:
    AprilTagFinder(std::shared_ptr<Turret> &turret, std::shared_ptr<Drivetrain> drivetrain);
    class VisionMeasurement
    {
    public:
        int _tagID;
        frc::Pose2d _pose;              // this is in field coordinates
        frc::Transform2d _relativePose; // this is in robot coordinates.
        units::second_t _timeStamp;     // In seconds I'm not sure yet
        wpi::array<double, 3U> _stddevs; // Error in this measurement (standard deviations)
        
        VisionMeasurement(frc::Pose2d pose, frc::Transform2d relativePose, units::second_t timeStamp, int tagID, const wpi::array<double, 3U> &stddevs);
    };
    struct RobotCamera{
    public: 
        std::shared_ptr<photon::PhotonCamera> _camera;
        frc::Transform3d _transform;
        bool _isTurret;
        RobotCamera(std::shared_ptr<photon::PhotonCamera> camera, frc::Transform3d transform, bool isTurret = false) : _camera(camera), _transform(transform), _isTurret(isTurret){};
    };
    
    static frc::Pose3d estimateFieldToRobotAprilTag(frc::Transform3d cameraToTarget, frc::Pose3d fieldRelativeTagPose, frc::Transform3d robotToCamera);

    std::vector<VisionMeasurement> getAllMeasurements();

    std::vector<photon::PhotonTrackedTarget> getCamTargets(std::shared_ptr<photon::PhotonCamera> camera);

    frc::Transform2d toTransform2d(frc::Transform3d t3d);

    std::vector<VisionMeasurement> getCamMeasurements(std::vector<photon::PhotonPipelineResult> results, frc::Transform3d camTransform3d);
    
    frc::Transform3d getRobotCam(int index);

    void Periodic() override;
    
    static std::vector<RobotCamera> _cameras;
    
    private:

        const double ambiguityThreshold = 0.28;
        // Base stddevs for measurements:
        wpi::array<double, 3U> base_stddevs = {0.5, 0.5, 0.5};
        // Use tag range to estimate measurement errors:
        wpi::array<double, 3U> estimate_stddevs(units::length::meter_t range);

        // Ignore things farther away than this.
        units::meter_t max_range{3.5};

        std::vector<VisionMeasurement> _visionMeasurements;
        std::shared_ptr<Turret> m_turret;
        std::shared_ptr<Drivetrain> m_drivetrain;

};