#include "subsystems/TargetFinder.h"

// TODO: Fix.
const frc::Pose2d TargetFinder::FIELD_CENTER = frc::Pose2d(291.00_in, 158.32_in, frc::Rotation2d());

//AndyMark Perimeter
const frc::Pose2d TargetFinder::BLUEHUB = frc::Pose2d(181.56_in, 158.32_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::REDHUB = frc::Pose2d(468.56_in, 158.32_in, frc::Rotation2d());

//Welded Perimeter
/*
const frc::Pose2d TargetFinder::BLUEHUB = frc::Pose2d(182.11_in, 158.84_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::REDHUB = frc::Pose2d(469.11_in, 158.84_in, frc::Rotation2d());
*/

const frc::Transform2d TargetFinder::ROBOTOTURRET = frc::Transform2d(-4.391_in, -7.409_in, frc::Rotation2d());

//literally the most estimated values ever
const frc::Pose2d TargetFinder::REDPASS_R = frc::Pose2d(557.5_in, 237.5_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::REDPASS_L = frc::Pose2d(557.5_in, 80_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::BLUEPASS_R = frc::Pose2d(91_in, 80_in, frc::Rotation2d());
const frc::Pose2d TargetFinder::BLUEPASS_L = frc::Pose2d(91_in, 237.5_in, frc::Rotation2d());




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
    else
    {
        target = Pass();
    }
    
    return target;
}

frc::Pose2d TargetFinder::getHubPos()
{
    //all in field coordinates
    frc::Pose2d TargetLoc = OurHub;

    auto tempLoc = TargetLoc.RelativeTo(turretPos);
    auto tempRange = tempLoc.Translation().Norm();

    // auto shot = BallisticShot::GetShot(tempRange, 1.5_m); new
    // frc::Translation2d velocityOffset (_localizer->getSpeeds().vx * shot.ShotTime, _localizer->getSpeeds().vy * shot.ShotTime); new
    frc::Translation2d velocityOffset (_localizer->getSpeeds().vx * 1_s, _localizer->getSpeeds().vy * 1_s);
    
    //TODO: Fix scaling offset
    // auto time = (tempRange / 0.744_mps); Over Compenstated
    // auto time = (tempRange / 1.488_mps);
    // auto time = (tempRange / 2_mps);
    // auto time = (tempRange / 4_mps);
    double A = 0.1;
    double B = 0.0;
    double C = 1.5;
    double time = ((tempRange*A) + ((tempRange*tempRange)*B) + C); // Emperical Model

    //velocityOffset = -velocityOffset * time;

    //TargetLoc = TargetLoc.TransformBy(frc::Transform2d(-velocityOffset, frc::Rotation2d())); new
    TargetLoc = TargetLoc.TransformBy(frc::Transform2d(velocityOffset, frc::Rotation2d()));

    //turns into robo coordinates
    return TargetLoc.RelativeTo(turretPos);
}

//right and left are swapped for red alliance bc zones are from blue alliance perspective
frc::Pose2d TargetFinder::Pass()
{
    UpdateAlliance();
    if (_alliance.value() == frc::DriverStation::Alliance::kRed && zone.contains("RIGHTHALF"))
    {
        return REDPASS_L.frc::Pose2d::RelativeTo(turretPos);
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kRed && zone.contains("LEFTHALF"))
    {
        return REDPASS_R.frc::Pose2d::RelativeTo(turretPos);
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue && zone.contains("RIGHTHALF"))
    {
        return BLUEPASS_R.frc::Pose2d::RelativeTo(turretPos);
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue && zone.contains("LEFTHALF"))
    {
        return BLUEPASS_L.frc::Pose2d::RelativeTo(turretPos);
    }

    return FIELD_CENTER.frc::Pose2d::RelativeTo(turretPos);
}


void TargetFinder::Periodic(){
    UpdateAlliance();

    zone = _zonefinder->GetZones();
    turretPos = _localizer->getPose().TransformBy(ROBOTOTURRET);

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