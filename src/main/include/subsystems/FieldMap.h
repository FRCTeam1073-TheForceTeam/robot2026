// MAP: loads the map
#pragma once

#include <frc2/command/SubsystemBase.h>


#include <photon/targeting/PhotonTrackedTarget.h>

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>

#include <frc/smartdashboard/SmartDashboard.h>


class FieldMap
{
    // TODO: AprilTagFields has two flavors of reefscape map: andymark and welded. which one do you need? who knows!
    // apparently for the 2/15 event it's welded. after that all bets are off.
    
    //temp comment out Aedan 
    
    public:
    static const frc::AprilTagFieldLayout fieldMap;

    units::meter_t findDistance(frc::Pose2d robot2DPose, int tagID);

    int getBestAprilTagID(frc::Pose2d robotPose);

    frc::Pose2d getTagRelativePose(int tagID, int slot, frc::Transform2d offset);
};