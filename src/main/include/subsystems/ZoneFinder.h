#pragma once
#include <frc/geometry/Rectangle2d.h>
#include <frc/geometry/Translation2d.h>
#include "subsystems/Localizer.h"
#include <frc/geometry/Pose2d.h>
#include <string>
#include <units/length.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <set>
#include <frc/DriverStation.h>



class ZoneFinder : public frc2::SubsystemBase
{
    public:
    class Zone 
    {
        public:

        Zone() = default;
        Zone(const std::string &n, const frc::Rectangle2d &r) : name(n), rect(r) {};
        std::string name;
        frc::Rectangle2d rect;

    };
    using ZoneVector = std::vector<Zone>;


    ZoneFinder(std::shared_ptr<Localizer>& localizer);

    std::set<std::string> GetZones();
    void Periodic() override;

    private:
    std::optional<frc::DriverStation::Alliance> _alliance;
    std::shared_ptr<Localizer> _localizer;
    frc::Translation2d CurrentTrans;
    ZoneVector zones;
};