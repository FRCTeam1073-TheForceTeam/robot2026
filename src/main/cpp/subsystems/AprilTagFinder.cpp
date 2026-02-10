#include "subsystems/AprilTagFinder.h"
std::vector<AprilTagFinder::RobotCamera> AprilTagFinder::_cameras = {};
AprilTagFinder::AprilTagFinder()
{
    std::cout << "Creating April Tag Object" << std::endl;
    _cameras = {
        RobotCamera(std::make_shared<photon::PhotonCamera>("Back_Left"), frc::Transform3d(frc::Translation3d(-0.2_m, 0.239_m, 0.1_m),frc::Rotation3d(0_deg, -15_deg, -150_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Back_Right"), frc::Transform3d(frc::Translation3d(-0.2_m, -0.239_m, 0.1_m),frc::Rotation3d(0_deg, -15_deg, 150_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Front_Left"), frc::Transform3d(frc::Translation3d(0.2_m, 0.239_m, 0.1_m),frc::Rotation3d(0_deg, -15_deg, -30_deg))),
        RobotCamera(std::make_shared<photon::PhotonCamera>("Front_Right"), frc::Transform3d(frc::Translation3d(0.2_m, -0.239_m, 0.1_m),frc::Rotation3d(0_deg, -15_deg, 30_deg)))
    };
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

std::vector<AprilTagFinder::VisionMeasurement> AprilTagFinder::getCamMeasurements(std::shared_ptr<photon::PhotonCamera> camera, frc::Transform3d camTransform3d) {
    std::vector<VisionMeasurement> measurements = std::vector<VisionMeasurement>();
    std::vector<photon::PhotonPipelineResult> results = camera->GetAllUnreadResults();

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

void AprilTagFinder::Periodic() {
    _visionMeasurements.clear();
    int i = 0;
    for (auto& cam : _cameras) {
        std::vector<AprilTagFinder::VisionMeasurement> measurements = getCamMeasurements(cam._camera, cam._transform);
        // for(auto& vm : measurements){
        //     std::cout << "Cam: " << i << std::endl;
        //     std::cout << "ID: " << vm._tagID << std::endl;
        //     std::cout << "Range: " << vm._range.value() << std::endl;
        //     std::cout << "Time: " << vm._timeStamp.value() << std::endl;
        // }
        _visionMeasurements.insert(
            _visionMeasurements.end(),
            measurements.begin(),
            measurements.end()
        );
        i++;
    }
}

wpi::array<double, 3U> AprilTagFinder::estimate_stddevs(units::length::meter_t range) {
    wpi::array<double, 3U> result(base_stddevs);

    result[0] += 0.2 * range.value();
    result[1] += 0.2 * range.value();
    result[2] += 0.15 * range.value();
    return result;
}
