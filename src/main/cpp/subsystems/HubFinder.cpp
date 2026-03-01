#include "subsystems/HubFinder.h"

//AndyMark Perimeter
const frc::Pose2d HubFinder::BLUEHUB = frc::Pose2d(181.56_in, 158.32_in, frc::Rotation2d());
const frc::Pose2d HubFinder::REDHUB = frc::Pose2d(468.56_in, 158.32_in, frc::Rotation2d());

//Welded Perimeter
/*
const frc::Pose2d HubFinder::BLUEHUB = frc::Pose2d(182.11_in, 158.84_in, frc::Rotation2d());
const frc::Pose2d HubFinder::REDHUB = frc::Pose2d(469.11_in, 158.84_in, frc::Rotation2d());
*/

const frc::Transform2d HubFinder::ROBOTOTURRET = frc::Transform2d(4.902_in, -5.375_in, frc::Rotation2d());




HubFinder::HubFinder(std::shared_ptr<Localizer> localizer):_localizer(localizer){
    feedback.rangeToHub = 0_m;
    feedback.turretToHubAngle = 0_rad;
}


frc::Pose2d HubFinder::getHubPos()
{
    frc::Pose2d HubLoc = OurHub.frc::Pose2d::RelativeTo(roboPos);
    return HubLoc;
}


void HubFinder::Periodic(){
    UpdateAlliance();
    roboPos = _localizer->getPose();
    frc::Pose2d turretLoc = getHubPos().TransformBy(ROBOTOTURRET);
    auto relativeHubPos = turretLoc.Translation();
    auto angle = units::math::atan2(relativeHubPos.Y(), relativeHubPos.X());
    feedback.turretToHubAngle = angle;
    feedback.rangeToHub = relativeHubPos.Norm();
    frc::SmartDashboard::PutNumber("Hubfinder/Turret Angle", feedback.turretToHubAngle.value());
    frc::SmartDashboard::PutNumber("Hubfinder/Turret Range", feedback.rangeToHub.value());
}

void HubFinder::UpdateAlliance(){
    _alliance = frc::DriverStation::GetAlliance();

    if (_alliance.value() == frc::DriverStation::Alliance::kRed){
        OurHub = REDHUB;
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue){
        OurHub = BLUEHUB;
    }
    else{
        std::cout << "HubFinder::No Alliance Selected";
    }
}