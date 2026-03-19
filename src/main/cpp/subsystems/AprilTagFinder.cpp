#include "subsystems/AprilTagFinder.h"
std::vector<AprilTagFinder::RobotCamera> AprilTagFinder::_cameras = {};
AprilTagFinder::AprilTagFinder(std::shared_ptr<Turret> &turret, std::shared_ptr<Drivetrain> drivetrain) : 
    m_turret(turret),
    m_drivetrain(drivetrain),
    _estimators({})
{
    std::cout << "Creating April Tag Object" << std::endl;
    _cameras = {
        RobotCamera(std::make_shared<photon::PhotonCamera>("Left_Front"), frc::Transform3d(frc::Translation3d(-8.974_in, 8.454_in, 4.896_in),frc::Rotation3d(0_deg, -21_deg, 65_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Left_Back"), frc::Transform3d(frc::Translation3d(-10.864_in, 7.858_in, 6.896_in),frc::Rotation3d(0_deg, -21_deg, 150_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Right_Front"), frc::Transform3d(frc::Translation3d(-8.974_in, -13.454_in, 4.896_in),frc::Rotation3d(0_deg, -21_deg, -65_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Right_Back"), frc::Transform3d(frc::Translation3d(-10.864_in, -10.235522_in, 6.896_in),frc::Rotation3d(0_deg, -21_deg, -150_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Turret"), frc::Transform3d(frc::Translation3d(-4.373_in, -12.858_in, 18.35_in),frc::Rotation3d(0_deg, 0_deg, 0_deg)), true) //TODO: Change Numbers
    };
    for(auto& camera : _cameras)
    {
        _estimators.push_back(photon::PhotonPoseEstimator(FieldMap::fieldMap,camera._transform));
    }
}

frc::Pose3d AprilTagFinder::estimateFieldToRobotAprilTag(frc::Transform3d cameraToTarget, frc::Pose3d fieldRelativeTagPose, frc::Transform3d robotToCamera) {
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

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getAllMeasurements() {
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

frc::Transform2d AprilTagFinder::toTransform2d(frc::Transform3d t3d) {
    return frc::Transform2d(t3d.X(), t3d.Y(), t3d.Rotation().ToRotation2d());
}

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getCamMeasurements(std::vector<photon::PhotonPipelineResult> results, frc::Transform3d camTransform3d) {
    std::vector<VisionMeasurement> measurements = std::vector<VisionMeasurement>();
    for (auto& result : results){
        if (result.HasTargets()){
        // targets.addAll(result.getTargets());
        units::time::second_t result_timestamp = result.GetTimestamp(); // Adjusted for each result for time compensation.

        for (auto& target : result.GetTargets()) {
            
            if (FieldMap::fieldMap.GetTagPose(target.GetFiducialId()).has_value()){
                if (target.GetPoseAmbiguity() != -1 && target.GetPoseAmbiguity() < ambiguityThreshold){
                    frc::Transform3d best = target.GetBestCameraToTarget();
                    frc::Pose3d robotPose = estimateFieldToRobotAprilTag(best,
                        FieldMap::fieldMap.GetTagPose(target.GetFiducialId()).value(), 
                        camTransform3d);
                    // In robot coordinates:
                    frc::Transform2d relativePose = toTransform2d(camTransform3d+best);
                    units::length::meter_t range = relativePose.Translation().Norm();
                    
                    // Ignore things that are too far away:
                    if (range < max_range) {
                        // TODO: Estimated STD Deviations from Photon vision:
                        auto std_devs = estimate_stddevs(range);

                        measurements.push_back(VisionMeasurement(robotPose.ToPose2d(), relativePose, result_timestamp, 
                                                                target.GetFiducialId(), std_devs));
                    }
                }
            }
        }
      }
    }
    return measurements;
}

frc::Transform3d AprilTagFinder::getRobotCam(int index) {
    return _cameras[index]._transform;
}

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getMultiTagEstimate(std::vector<photon::PhotonPipelineResult> results, photon::PhotonPoseEstimator& estimator, frc::Transform3d camTransform3d)
{
    estimator.SetRobotToCameraTransform(camTransform3d);
    std::vector<VisionMeasurement> measurements = std::vector<VisionMeasurement>();
    for(auto& result : results)
    {
        std::optional<photon::EstimatedRobotPose> pose = estimator.EstimateCoprocMultiTagPose(result);
        if(!pose.has_value())
        {
            pose = estimator.EstimateLowestAmbiguityPose(result);
            if(!pose.has_value())
                continue;
        }
        auto estimated_pose = pose.value();
        units::length::meter_t minDist = 100.0_m;
        for(auto& t : estimated_pose.targetsUsed)
            minDist = units::math::min(minDist,units::length::meter_t(
                t.GetBestCameraToTarget().Translation().Norm()
            ));
        
        auto std_devs = estimate_stddevs(minDist); //TODO: find the actual value
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
    for (int i = 0; i<_cameras.size(); i++) {
        auto& cam = _cameras[i];
        auto& estimator = _estimators[i];
        if(cam._isTurret && (!m_turret->GetFeedback().haveZero || m_turret->GetFeedback().velocity>2_rad_per_s))
            continue;
        std::vector<photon::PhotonPipelineResult> results = cam._camera->GetAllUnreadResults();
        frc::Transform3d transform = cam._transform;
        //If the camera is the turrets camera, and the velocty of the turret is acceptable we will use it. And if not we will skip over using the camera.
        //TODO: Revisit threshold
        if (cam._isTurret && std::abs(turretVelocity.value()) < 1.0){
            units::time::millisecond_t totalLatency = 0_ms;
            float count = 0.0;
            for (auto &result : results){
                totalLatency += result.GetLatency();
                count = count + 1.0;
            }
            units::time::millisecond_t averageLatency = 0_ms;
            if(count > 0.0){
                averageLatency = totalLatency / count;
            }

            auto turretAngle = m_turret->GetFeedback().position - turretVelocity * averageLatency; //TODO: Tweak this number
            transform = (transform + frc::Transform3d(frc::Translation3d(), frc::Rotation3d(0_deg, 0_deg, turretAngle))) + (frc::Transform3d(frc::Translation3d(0_in, -6.250_in, 0_in), frc::Rotation3d(0_deg, -15_deg, 0_deg)));
        }

        //std::vector<AprilTagFinder::VisionMeasurement> measurements = getCamMeasurements(results, transform);
        estimator.AddHeadingData(m_drivetrain->GetPreviousUpdateTime(),m_drivetrain->GetGyroHeading());
        std::vector<AprilTagFinder::VisionMeasurement> measurements = getMultiTagEstimate(results, estimator, transform);
        _visionMeasurements.insert(
            _visionMeasurements.end(),
            measurements.begin(),
            measurements.end()
        );
    }
    frc::SmartDashboard::PutBoolean("AprilTagFinder/HasAprilTags", hasAprilTags);
}

wpi::array<double, 3U> AprilTagFinder::estimate_stddevs(units::length::meter_t range) {
    wpi::array<double, 3U> result(base_stddevs);

    result[0] += 0.2 * range.value();
    result[1] += 0.2 * range.value();
    result[2] += 0.15 * range.value();
    return result;
}