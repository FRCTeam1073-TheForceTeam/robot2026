#include "subsystems/ZoneFinder.h"

const frc::Rectangle2d ZoneFinder::REDZONE = frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(0_in, 0_in));
const frc::Rectangle2d ZoneFinder::BLUEZONE = frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(317.69_in, 182.125_in));
const frc::Rectangle2d ZoneFinder::NEUTRALZONE = frc::Rectangle2d(frc::Translation2d(0_in, 182.125_in), frc::Translation2d(0_in, 0_in));
const frc::Rectangle2d ZoneFinder::UNKNOWN = frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(324_in, 648_in));

ZoneFinder::ZoneFinder(std::shared_ptr<Localizer> _localizer)
{
    frc::Translation2d CurrentTrans = _localizer->getPose().frc::Pose2d::Translation();
};

frc::Rectangle2d ZoneFinder::GetZone()
{
if(REDZONE.frc::Rectangle2d::Contains(CurrentTrans))
{
    return REDZONE;
}
else if(BLUEZONE.frc::Rectangle2d::Contains(CurrentTrans))
{
    return BLUEZONE;
}
else if(NEUTRALZONE.frc::Rectangle2d::Contains(CurrentTrans))
{
    return NEUTRALZONE;
}
else
{
    return UNKNOWN;
}
};