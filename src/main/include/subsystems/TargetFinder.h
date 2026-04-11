#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <string>
#include <iostream>
#include <units/time.h>

#include "subsystems/Localizer.h"
#include "subsystems/ZoneFinder.h"


class TargetFinder  : public frc2::SubsystemBase {
    public:
    static const frc::Pose2d FIELD_CENTER;
    static const frc::Pose2d REDHUB;
    static const frc::Pose2d BLUEHUB;
    static const frc::Pose2d REDPASS_R;
    static const frc::Pose2d REDPASS_L;
    static const frc::Pose2d BLUEPASS_R;
    static const frc::Pose2d BLUEPASS_L;
    static const frc::Transform2d ROBOTOTURRET;
    
    struct Feedback 
    {
        units::angle::radian_t turretToTargetAngle;
        units::length::meter_t rangeToTarget;
        bool passing; // Is this a passing target?
    };

    TargetFinder(std::shared_ptr<Localizer>& localizer, std::shared_ptr<ZoneFinder>& zonefinder);

    frc::Pose2d getTargetPos();
    frc::Pose2d _target;
    bool _passing;

    frc::Pose2d getHubPos();
    frc::Pose2d _targetLoc;

    frc::Pose2d Pass();
    
    const Feedback& getFeedback(){
        return feedback;
    }

    void Periodic() override;
    

    private:
    std::shared_ptr<Localizer> _localizer;
    std::shared_ptr<ZoneFinder> _zonefinder;

    frc::Pose2d OurHub;
    frc::Pose2d turretPos;
    frc::Pose2d VirtualHub;

    Feedback feedback;

    void UpdateAlliance();
    std::optional<frc::DriverStation::Alliance> _alliance;
    std::set<std::string> zone;
    std::string OurZone;
};