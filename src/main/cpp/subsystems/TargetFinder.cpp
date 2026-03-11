#include "subsystems/TargetFinder.h"

//AndyMark Perimeter
const frc::Pose2d TargetFinder::BLUEHUB = frc::Pose2d(181.56_in, 158.32_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::REDHUB = frc::Pose2d(468.56_in, 158.32_in, frc::Rotation2d());

//Welded Perimeter
/*
const frc::Pose2d TargetFinder::BLUEHUB = frc::Pose2d(182.11_in, 158.84_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::REDHUB = frc::Pose2d(469.11_in, 158.84_in, frc::Rotation2d());
*/

const frc::Transform2d TargetFinder::ROBOTOTURRET = frc::Transform2d(4.902_in, -5.375_in, frc::Rotation2d());

const frc::Pose2d TargetFinder::REDPASS_R = frc::Pose2d(0_in, 0_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::REDPASS_L = frc::Pose2d(0_in, 0_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::BLUEPASS_R = frc::Pose2d(0_in, 0_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::BLUEPASS_L = frc::Pose2d(0_in, 0_in, frc::Rotation2d());




TargetFinder::TargetFinder(std::shared_ptr<Localizer>& localizer, std::shared_ptr<ZoneFinder>& zonefinder):
    _localizer(localizer), 
    _zonefinder(zonefinder) {
    feedback.rangeToTarget = 0_m;
    feedback.turretToTargetAngle = 0_rad;
}

frc::Pose2d TargetFinder::getTargetPos()
{
    if(_zonefinder->GetZones().contains(OurZone))
    {
        target = getHubPos();
    }
    else if(_zonefinder->GetZones().contains("NEUTRALZONE"))
    {
        target = Pass();
    }
    else
    {
        //TODO: figure out what to put here. Right now it's the center of the welded field
        target = frc::Pose2d (325.61_in, 158.84_in, frc::Rotation2d());
    }
    return target;
}

frc::Pose2d TargetFinder::getHubPos()
{
    //all in field coordinates
    frc::Pose2d TargetLoc = OurHub;

    frc::Translation2d velocityOffset (_localizer->getSpeeds().vx * 1_s, _localizer->getSpeeds().vy * 1_s);
    //TODO: Fix scaling offset
    velocityOffset = velocityOffset * 0.0;
    TargetLoc.TransformBy(frc::Transform2d(velocityOffset, frc::Rotation2d()));

    //turns into robo coordinates
    return TargetLoc.RelativeTo(roboPos);
}

//right and left are swapped for red alliance bc zones are from blue alliance perspective
frc::Pose2d TargetFinder::Pass()
{
    UpdateAlliance();
    if (_alliance.value() == frc::DriverStation::Alliance::kRed && zone.contains("RIGHTHALF"))
    {
        return REDPASS_L.frc::Pose2d::RelativeTo(roboPos);
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kRed && zone.contains("LEFTHALF"))
    {
        return REDPASS_R.frc::Pose2d::RelativeTo(roboPos);
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue && zone.contains("RIGHTHALF"))
    {
        return REDPASS_L.frc::Pose2d::RelativeTo(roboPos);
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue && zone.contains("LEFTHALF"))
    {
        return REDPASS_L.frc::Pose2d::RelativeTo(roboPos);
    }
}


void TargetFinder::Periodic(){
    UpdateAlliance();

    zone = _zonefinder->GetZones();
    roboPos = _localizer->getPose().TransformBy(ROBOTOTURRET);

    auto relativeTargetPos = getTargetPos().Translation();

    auto angle = units::math::atan2(relativeTargetPos.Y(), relativeTargetPos.X());
    feedback.turretToTargetAngle = angle;
    feedback.rangeToTarget = relativeTargetPos.Norm();
    frc::SmartDashboard::PutNumber("TargetFinder/Turret Angle", feedback.turretToTargetAngle.value());
    frc::SmartDashboard::PutNumber("TargetFinder/Turret Range", feedback.rangeToTarget.value());
}

void TargetFinder::UpdateAlliance(){
    _alliance = frc::DriverStation::GetAlliance();

    if (_alliance.value() == frc::DriverStation::Alliance::kRed){
        OurHub = REDHUB;
        OurZone = "REDZONE";
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue){
        OurHub = BLUEHUB;
        OurZone = "BLUEZONE";
    }
    else{
        std::cout << "TargetFinder::No Alliance Selected";
    }
}