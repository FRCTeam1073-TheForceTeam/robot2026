#include "subsystems/AprilTagFinder.h"
std::vector<AprilTagFinder::RobotCamera> AprilTagFinder::_cameras = {};
AprilTagFinder::AprilTagFinder(std::shared_ptr<Turret> &turret, std::shared_ptr<Drivetrain> drivetrain) : 
    m_turret(turret),
    m_drivetrain(drivetrain),
    _estimators({})
{
    std::cout << "Creating April Tag Object" << std::endl;
    //
    // Thesee are poses in robot coordinates:
    //  
    // Robot coordinates have +X forward, +Y out left of robot and +Z up (opposite of gravity)
    // The origin of X,Y is the geometric center of the robot frame perimeter.
    // The origin of Z is *ON THE FLOOR* it is a virtual point at zero field height that is not actually *inside* the robot. It is the
    // projection of the X,Y geometric center onto the floor.
    //
    // Rotations are angles about these primary axes usign the right-hand-rule.
    //
    // The turrent height is to the center of the turrent rotation bearing plane. Turrent location is center of turret rotation bearing.
    //
    // Center of pigeon height is 4.75in, offset from X,Y center of robot ()
    //

    const frc::Translation3d pigeon_offset(-1.0_in, 2.5_in, 4.75_in);

        _cameras = {
        RobotCamera(std::make_shared<photon::PhotonCamera>("Left_Front"), frc::Transform3d(frc::Translation3d(-8.977_in, 8.448_in, 5.152_in) + pigeon_offset, frc::Rotation3d(0_deg, -21_deg, 65_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Left_Back"), frc::Transform3d(frc::Translation3d(-10.858_in, 7.855_in, 7.562_in) + pigeon_offset, frc::Rotation3d(0_deg, -21_deg, 150_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Right_Front"), frc::Transform3d(frc::Translation3d(-8.977_in, -13.448_in, 5.152_in) + pigeon_offset,frc::Rotation3d(0_deg, -21_deg, -65_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Right_Back"), frc::Transform3d(frc::Translation3d(-10.858_in, -12.855_in, 7.652_in) + pigeon_offset,frc::Rotation3d(0_deg, -21_deg, -150_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Turret"), frc::Transform3d(frc::Translation3d(-4.39_in, -7.409_in, 12.0_in) + pigeon_offset,frc::Rotation3d(0_deg, 0_deg, 0_deg)), true) 
    };
    for(auto& camera : _cameras)
    {
        _estimators.push_back(photon::PhotonPoseEstimator(FieldMap::fieldMap,camera._transform));
    }
}

frc::Pose3d AprilTagFinder::estimateFieldToRobotAprilTag(const frc::Transform3d& cameraToTarget, const frc::Pose3d& fieldRelativeTagPose, const frc::Transform3d& robotToCamera) {
    return fieldRelativeTagPose+(cameraToTarget.Inverse())+(robotToCamera.Inverse());
}

AprilTagFinder::VisionMeasurement::VisionMeasurement(frc::Pose2d pose, frc::Transform2d relativePose, units::second_t timeStamp, int tagID,
     const wpi::array<double, 3U>& stddevs) :
    _tagID(tagID),
    _pose(pose),
    _relativePose(relativePose),
    _timeStamp(timeStamp),
    _stddevs(stddevs)
{}

const std::vector<AprilTagFinder::VisionMeasurement>& AprilTagFinder::getAllMeasurements() const {
    return _visionMeasurements;
}

std::vector<photon::PhotonTrackedTarget> AprilTagFinder::getCamTargets(std::shared_ptr<photon::PhotonCamera> camera) {
    std::vector<photon::PhotonPipelineResult> results = camera->GetAllUnreadResults();
    std::vector<photon::PhotonTrackedTarget> targets;

    for(auto& result : results){
        if(result.HasTargets()){
            //I don't know a good way of turning a std::span into an std::vector so I just did this
            std::span<const photon::PhotonTrackedTarget> r = result.GetTargets();
            for(auto& c : r)
            {
                targets.push_back(c);
            }
        }
    }
    return targets;
}

frc::Transform2d AprilTagFinder::toTransform2d(const frc::Transform3d& t3d) {
    return frc::Transform2d(t3d.X(), t3d.Y(), t3d.Rotation().ToRotation2d());
}

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getCamMeasurements(const std::vector<photon::PhotonPipelineResult>& results, 
                                                                                  const frc::Transform3d& camTransform3d) {
    std::vector<VisionMeasurement> measurements = std::vector<VisionMeasurement>();
    for (const auto& result : results) {
        if (result.HasTargets()){
        // targets.addAll(result.getTargets());
        units::time::second_t result_timestamp = result.GetTimestamp(); // Adjusted for each result for time compensation.

        for (const auto& target : result.GetTargets()) {
            
            if (FieldMap::fieldMap.GetTagPose(target.GetFiducialId()).has_value()){
                if (target.GetPoseAmbiguity() != -1 && target.GetPoseAmbiguity() < ambiguityThreshold){
                    frc::Transform3d best = target.GetBestCameraToTarget();
                    // Field coordinates:
                    frc::Pose3d robotPose = estimateFieldToRobotAprilTag(best,
                        FieldMap::fieldMap.GetTagPose(target.GetFiducialId()).value(), 
                        camTransform3d);

                    // In robot coordinates:
                    frc::Transform2d relativePose = toTransform2d(camTransform3d+best);
                    units::length::meter_t range = relativePose.Translation().Norm();
                    
                    // Ignore things that are too far away:
                    if (range < max_range) {
                        // TODO: Estimated STD Deviations from Photon vision:
                        auto std_devs = estimate_stddevs(range, relativePose.Rotation().Radians() + robotPose.Rotation().Z());

                        measurements.push_back(VisionMeasurement(robotPose.ToPose2d(), relativePose, result_timestamp, 
                                                                target.GetFiducialId(), std_devs));
                    }
                }
            }
        } // End loop over targets.
      }
    }
    return measurements;
}

const frc::Transform3d& AprilTagFinder::getRobotCamTransform(int index) const {
    return _cameras.at(index)._transform;
}

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getMultiTagEstimate(const std::vector<photon::PhotonPipelineResult>& results, 
                                                                                    photon::PhotonPoseEstimator& estimator, 
                                                                                    const frc::Transform3d& camTransform3d)
{
    estimator.SetRobotToCameraTransform(camTransform3d);
    std::vector<VisionMeasurement> measurements = std::vector<VisionMeasurement>();
    for (const auto& result : results)
    {
        // In field coordinates:
        std::optional<photon::EstimatedRobotPose> pose = estimator.EstimateCoprocMultiTagPose(result);

        if (!pose.has_value())
        {
            // TODO: This seems like redundant work if we're doing multi tag poses?
            pose = estimator.EstimateLowestAmbiguityPose(result);
            if (!pose.has_value())
                 continue;
            if (pose->targetsUsed.empty() || pose->targetsUsed[0].poseAmbiguity > ambiguityThreshold)
                 continue;
        }
        auto estimated_pose = pose.value();
        units::length::meter_t minDist = 100.0_m;
        units::angle::radian_t minAngle = 0.0_rad;
        for(auto& t : estimated_pose.targetsUsed) {
            const auto best = t.GetBestCameraToTarget();
            const auto dist = best.Translation().Norm();
            if (dist < minDist) {
                minDist = dist;
                minAngle = best.Rotation().Z(); // Yaw angle.
            }
        }

        auto std_devs = estimate_stddevs(minDist,  minAngle); // TODO: find the actual value
        hasAprilTags = true;
        measurements.push_back(VisionMeasurement(estimated_pose.estimatedPose.ToPose2d(),frc::Transform2d(),estimated_pose.timestamp,0,std_devs));
    }
    return measurements;
}

void AprilTagFinder::clearMeasurements() {
    _visionMeasurements.clear();
}

void AprilTagFinder::Periodic() {
    hasAprilTags = false;
    auto turretVelocity = m_turret->GetFeedback().velocity;
    for (size_t i = 0; i < _cameras.size(); i++) {
        const auto& cam = _cameras[i];
        auto& estimator = _estimators[i];

        std::vector<photon::PhotonPipelineResult> results = cam._camera->GetAllUnreadResults();
        frc::Transform3d transform = cam._transform;

        // If the camera is the turrets camera, and the velocty of the turret is acceptable we will use it. And if not we will skip over using the camera.
        if (cam._isTurret) {
            // If the camera is the turrent but it is not zeroed/indexed skip it.
            if (!m_turret->GetFeedback().haveZero) {
                frc::SmartDashboard::PutBoolean("AprilTagFinder/UsingTurretCam", false);
                continue;
            }

            // If the turret is moving too quickly then skip it otherwise try to use it.
            // TODO: Revisit threshold
            if (std::abs(turretVelocity.value()) < 1.0) {
                units::time::millisecond_t totalLatency = 0_ms;
                float count = 0.0;
                for (auto &result : results) {
                    totalLatency += result.GetLatency();
                    count = count + 1.0;
                }
                units::time::millisecond_t averageLatency = 0_ms;
                if (count > 0.0) {
                    averageLatency = totalLatency / count;
                }

                // Estimate turrent angle at point of average latency from measurements:
                auto turretAngle = m_turret->GetFeedback().position - turretVelocity * averageLatency; //TODO: Tweak this number
                transform = (transform + frc::Transform3d(frc::Translation3d(), frc::Rotation3d(0_deg, 0_deg, turretAngle))) + 
                            (frc::Transform3d(frc::Translation3d(-0.136_in, -6.125_in, 5.187_in), frc::Rotation3d(0_deg, -15_deg, 0_deg)));
                frc::SmartDashboard::PutBoolean("AprilTagFinder/UsingTurretCam", true);
            } else {
                frc::SmartDashboard::PutBoolean("AprilTagFinder/UsingTurretCam", false);
                continue; // Skip turret if it's moving too fast.
            }
        } // End camera is turret.

        // std::vector<AprilTagFinder::VisionMeasurement> measurements = getCamMeasurements(results, transform);
        estimator.AddHeadingData(m_drivetrain->GetPreviousUpdateTime(), m_drivetrain->GetGyroHeading());
        std::vector<AprilTagFinder::VisionMeasurement> measurements = getMultiTagEstimate(results, estimator, transform);
        _visionMeasurements.insert(
            _visionMeasurements.end(),
            measurements.begin(),
            measurements.end()
        );
    }
    frc::SmartDashboard::PutBoolean("AprilTagFinder/HasAprilTags", hasAprilTags);
}

wpi::array<double, 3U> AprilTagFinder::estimate_stddevs(units::length::meter_t range, units::angle::radian_t bearing) {
    wpi::array<double, 3U> result(base_stddevs);

    // TODO: Use bearing + range error model so that position errors are not symmertric.

    result[0] += 0.2 * range.value();
    result[1] += 0.2 * range.value();
    result[2] += 0.15 * range.value();
    return result;
}