#include <subsystems/FieldMap.h>

const frc::AprilTagFieldLayout FieldMap::fieldMap = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

units::meter_t FieldMap::findDistance(frc::Pose2d robot2DPose, int tagID) {

    std::optional<frc::Pose3d> tag3dPose = FieldMap::fieldMap.GetTagPose(tagID);

    if(!tag3dPose.has_value()) {
        //we don't have a value to work with, bail
        return 999_m;
    }

    units::meter_t tagX = tag3dPose.value().X();
    units::meter_t tagY = tag3dPose.value().Y();

    units::meter_t distance = units::math::sqrt((tagX-robot2DPose.X())*(tagX-robot2DPose.X())+(tagY-robot2DPose.Y())*(tagY-robot2DPose.Y()));
    return distance;
}

int FieldMap::getBestAprilTagID(frc::Pose2d robotPose) 
{
    units::meter_t shortestDistance = 998_m;

    std::vector<frc::AprilTag> aprilTags = fieldMap.GetTags();
    int bestID = -1;

    for(auto& tag : aprilTags) 
    {
        if (findDistance(robotPose, tag.ID) < shortestDistance && tag.ID != 4 && tag.ID != 5 && tag.ID != 14 && tag.ID != 15) 
        {
            shortestDistance = findDistance(robotPose, tag.ID);
            bestID = tag.ID;
        }
    }
    return bestID;
}

frc::Pose2d FieldMap::getTagRelativePose(int tagID, int slot, frc::Transform2d offset)
{
    frc::Pose2d tagPose = fieldMap.GetTagPose(tagID).value().ToPose2d();
    units::meter_t yOffset = 0.165_m;
    // double endEffectorOffset = 0.2286;
    units::meter_t endEffectorOffset = 0.1905_m;

    if(slot == -1) // left
    {
        tagPose = tagPose.TransformBy(frc::Transform2d(0_m, -yOffset + endEffectorOffset, frc::Rotation2d()));
    }
    else if(slot == 0) // center
    {
        tagPose = tagPose.TransformBy(frc::Transform2d(0_m, endEffectorOffset, frc::Rotation2d()));
    }
    else if(slot == 1) // right
    {
        tagPose = tagPose.TransformBy(frc::Transform2d(0_m, yOffset + endEffectorOffset, frc::Rotation2d()));
    }
    else if (slot == 2) // coral station
    {
        tagPose = tagPose.TransformBy(frc::Transform2d(0_m, -endEffectorOffset, frc::Rotation2d()));
    }

    tagPose = tagPose.TransformBy(offset);

    frc::SmartDashboard::PutNumber("FieldMap/TargetTagID", tagID);
    frc::SmartDashboard::PutNumber("FieldMap/TargetPoseX", tagPose.X().value());
    frc::SmartDashboard::PutNumber("FieldMap/TargetPoseY", tagPose.Y().value());
    frc::SmartDashboard::PutNumber("FieldMap/TargetPoseTheta", tagPose.Rotation().Radians().value());

    return tagPose;
}