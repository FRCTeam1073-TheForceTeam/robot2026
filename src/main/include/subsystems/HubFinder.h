#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>
#include "frc/geometry/Pose2d.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <frc/geometry/Rotation2d.h>


#include "subsystems/Localizer.h"



class HubFinder  : public frc2::SubsystemBase {
    public:
    static const frc::Pose2d REDHUB;
    static const frc::Pose2d BLUEHUB;
    static const frc::Transform2d ROBOTOTURRET;
    
    std::shared_ptr<Localizer> _localizer;
    HubFinder(std::shared_ptr<Localizer> localizer);

    void Periodic() override;

    frc::Pose2d getHubPos();
    units::angle::radian_t getTurretToHubAngle();

    private:
    void UpdateAlliance();
    
    std::optional<frc::DriverStation::Alliance> _alliance;

    frc::Pose2d HubLoc;
    frc::Pose2d TurretLoc;
    frc::Pose2d OurHub;
    frc::Pose2d RoboPos;
    frc::Rotation2d RoboRotation;

};