#include "subsystems/HubFinder.h"


const frc::Pose2d HubFinder::REDHUB = frc::Pose2d(486.57_in, 90_in, frc::Rotation2d());
const frc::Pose2d HubFinder::BLUEHUB = frc::Pose2d(223.67_in, 90_in, frc::Rotation2d());

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
    frc::Pose2d HubLoc = OurHub.frc::Pose2d::RelativeTo(RoboPos);
    return HubLoc;
};