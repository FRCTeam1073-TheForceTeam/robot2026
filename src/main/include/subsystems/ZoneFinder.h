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
    static const frc::Rectangle2d TRAVELZONE;
    //still figuring how to add 4 rectangle coordiantes(the bump+trench on each alliance) to the same name
    static const frc::Rectangle2d UNKNOWN;
    //unknown zone coordinates will just be the entire field

    ZoneFinder(std::shared_ptr<Localizer> _localizer);

    frc::Rectangle2d GetZone();
    //checks what zone robot is in based on localizer and returns Rectangle2d of the zone (Eva)


    private:
    frc::Translation2d CurrentTrans;
};