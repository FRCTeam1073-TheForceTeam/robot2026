#include "subsystems/TargetFinder.h"
#include "subsystems/BallisticShot.h"

// TODO: Fix.
const frc::Pose2d TargetFinder::FIELD_CENTER = frc::Pose2d(291.00_in, 158.32_in, frc::Rotation2d());

// AndyMark Perimeter
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


void TargetFinder::SetBallisticShot(std::shared_ptr<BallisticShot>& m_ballisticShot)
{
    _ballisticShot = m_ballisticShot;
}

TargetFinder::TargetFinder(std::shared_ptr<Localizer>& localizer, std::shared_ptr<ZoneFinder>& zonefinder):
    _localizer(localizer), 
    _zonefinder(zonefinder),
    _passing(false) {
    feedback.rangeToTarget = 0_m;
    feedback.turretToTargetAngle = 0_rad;
    feedback.passing = false;
}

frc::Pose2d TargetFinder::getTargetPos()
{
    if(_zonefinder->GetZones().contains(OurZone))
    {
        _target = getHubPos();
        _passing = false;
    }
    else
    {
        _target = Pass();
        _passing = true;
    }
    
    return _target;
}

frc::Pose2d TargetFinder::getHubPos()
{
    // All in field coordinates
    frc::Pose2d TargetLoc = OurHub;

    auto tempLoc = TargetLoc.RelativeTo(turretPos);
    auto tempRange = tempLoc.Translation().Norm();

    auto shot = _ballisticShot->GetShot();
    frc::Translation2d velocityOffset (_localizer->getSpeeds().vx * (shot.ShotTime), _localizer->getSpeeds().vy * (shot.ShotTime)); 
   
    // TODO: Use cross-product to compute relative velocity induced by rotation and add that term as well...
    auto rotationOffset = tempLoc.Translation() * _localizer->getSpeeds().omega.value();
    velocityOffset.operator+(rotationOffset) * 1.0;
    TargetLoc = TargetLoc.TransformBy(frc::Transform2d(-velocityOffset, frc::Rotation2d()));

    //turns into robo coordinates
    return TargetLoc.RelativeTo(turretPos);
}

//right and left are swapped for red alliance bc zones are from blue alliance perspective
frc::Pose2d TargetFinder::Pass()
{
    UpdateAlliance();
    if (_alliance.has_value()) {
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
    }

    // If we don't know anything else.
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
    feedback.passing = _passing;
    frc::SmartDashboard::PutNumber("TargetFinder/Turret Angle", feedback.turretToTargetAngle.value());
    frc::SmartDashboard::PutNumber("TargetFinder/Turret Range", feedback.rangeToTarget.value());
}

void TargetFinder::UpdateAlliance(){
    _alliance = frc::DriverStation::GetAlliance();

    if (_alliance.has_value()) {
        if (_alliance.value() == frc::DriverStation::Alliance::kRed){
            OurHub = REDHUB;
            OurZone = "REDZONE";
        }
        else if (_alliance.value() == frc::DriverStation::Alliance::kBlue){
            OurHub = BLUEHUB;
            OurZone = "BLUEZONE";
        }
    } else{
        std::cout << "TargetFinder::No Alliance Selected";
    }
}