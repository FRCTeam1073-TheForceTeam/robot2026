#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>
#include "frc/geometry/Pose2d.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>

#include "subsystems/Localizer.h"





class HubFinder  : public frc2::SubsystemBase {
    public:
    static const frc::Pose2d REDHUB;
    static const frc::Pose2d BLUEHUB;
    static const frc::Transform2d ROBOTOTURRET;
    
    struct Feedback {
        units::angle::radian_t turretToHubAngle;
        units::length::meter_t rangeToHub;
    };

    HubFinder(std::shared_ptr<Localizer> localizer);
    std::shared_ptr<Localizer> _localizer;

    frc::Pose2d getHubPos();
    frc::Pose2d HubLoc;
    
    const Feedback& getFeedback(){
        return feedback;
    }

    void Periodic();


    

    private:
    frc::Pose2d OurHub;
    frc::Pose2d roboPos;

    Feedback feedback;

    void UpdateAlliance();
    std::optional<frc::DriverStation::Alliance> _alliance;

};