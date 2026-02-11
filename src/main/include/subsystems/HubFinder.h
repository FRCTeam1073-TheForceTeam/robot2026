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
    
    std::shared_ptr<Localizer> _localizer;
    HubFinder(std::shared_ptr<Localizer> localizer);

    frc::Pose2d HubLoc;
    
    frc::Pose2d getHubPos();

    private:
    static std::optional<frc::DriverStation::Alliance> _alliance;
    frc::Pose2d OurHub;
    frc::Pose2d RoboPos;

};