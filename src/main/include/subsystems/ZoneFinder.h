#include <frc/geometry/Rectangle2d.h>
#include <frc/geometry/Translation2d.h>
#include "subsystems/Localizer.h"
#include <frc/geometry/Pose2d.h>
#include <string>
#include <units/length.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>


class ZoneFinder
{
    public:
    static const frc::Rectangle2d REDZONE;
    static const frc::Rectangle2d BLUEZONE;
    static const frc::Rectangle2d NEUTRALZONE;
    
    static const frc::Rectangle2d TRENCH_A;
    static const frc::Rectangle2d TRENCH_B;
    static const frc::Rectangle2d TRENCH_C;
    static const frc::Rectangle2d TRENCH_D;
    
    static const frc::Rectangle2d BUMP_A;
    static const frc::Rectangle2d BUMP_B;
    static const frc::Rectangle2d BUMP_C;
    static const frc::Rectangle2d BUMP_D;


    ZoneFinder(std::shared_ptr<Localizer> _localizer);

    std::string GetZone();
    //checks what zone robot is in based on localizer and returns Rectangle2d of the zone (Eva)


    private:
    frc::Translation2d CurrentTrans;
};