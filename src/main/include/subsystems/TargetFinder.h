#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>
#include "frc/geometry/Pose2d.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <string>
#include <iostream>

#include "subsystems/Localizer.h"
#include "subsystems/ZoneFinder.h"





class TargetFinder  : public frc2::SubsystemBase {
    public:
    static const frc::Pose2d REDHUB;
    static const frc::Pose2d BLUEHUB;
    static const frc::Pose2d REDPASS_R;
    static const frc::Pose2d REDPASS_L;
    static const frc::Pose2d BLUEPASS_R;
    static const frc::Pose2d BLUEPASS_L;
    static const frc::Transform2d ROBOTOTURRET;
    
    struct Feedback {
        units::angle::radian_t turretToHubAngle;
        units::length::meter_t rangeToHub;
        frc::Translation2d RedPass_R;
        frc::Translation2d RedPass_L;
        frc::Translation2d BluePass_R;
        frc::Translation2d BluePass_L;
    };

    TargetFinder(std::shared_ptr<Localizer> localizer);
    std::shared_ptr<Localizer> _localizer;

    frc::Pose2d getHubPos();
    frc::Pose2d HubLoc;

    frc::Pose2d Pass();
    
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

    ZoneFinder zonefinder(std::shared_ptr<Localizer> _localizer);
    std::string _zone;

};