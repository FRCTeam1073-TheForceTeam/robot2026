#include "subsystems/HubFinder.h"

//AndyMark Perimeter
const frc::Pose2d HubFinder::BLUEHUB = frc::Pose2d(181.56_in, 158.32_in, frc::Rotation2d());
const frc::Pose2d HubFinder::REDHUB = frc::Pose2d(468.56_in, 158.32_in, frc::Rotation2d());

//Welded Perimeter
/*
const frc::Pose2d HubFinder::BLUEHUB = frc::Pose2d(182.11_in, 158.84_in, frc::Rotation2d());
const frc::Pose2d HubFinder::REDHUB = frc::Pose2d(469.11_in, 158.84_in, frc::Rotation2d());
*/

HubFinder::HubFinder(std::shared_ptr<Localizer> localizer):_localizer(localizer){
    frc::Pose2d RoboPos = _localizer->getPose();
    std::optional<frc::DriverStation::Alliance> _alliance = frc::DriverStation::GetAlliance();
    if (_alliance.value() == frc::DriverStation::Alliance::kRed){
        frc::Pose2d OurHub = REDHUB;
    }
    else if (_alliance.value() == frc::DriverStation::Alliance::kBlue){
        frc::Pose2d OurHub = BLUEHUB;
    }
    else{
        std::cout << "No Alliance Selected";
    }
};

frc::Pose2d HubFinder::getHubPos()
{
    // TODO: Need to check alliance every time here, because it is not set on construction.
    frc::Pose2d HubLoc = OurHub.frc::Pose2d::RelativeTo(RoboPos);
    return HubLoc;
};