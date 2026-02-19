#include "subsystems/ZoneFinder.h"

//AndyMark Dimentions
const frc::Rectangle2d ZoneFinder::BLUEZONE = frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(181.56_in, 316.64_in));

const frc::Rectangle2d ZoneFinder::REDZONE = frc::Rectangle2d(frc::Translation2d(468.56_in, 0_in), frc::Translation2d(650.12_in, 316.64_in));
const frc::Rectangle2d ZoneFinder::NEUTRALZONE = frc::Rectangle2d(frc::Translation2d(181.56_in, 0_in), frc::Translation2d(468.56_in, 316.64_in));

const frc::Rectangle2d ZoneFinder::TRENCH_A = frc::Rectangle2d(frc::Translation2d(156.06_in, 0_in), frc::Translation2d(200.46_in, 49.86_in));
const frc::Rectangle2d ZoneFinder::TRENCH_B = frc::Rectangle2d(frc::Translation2d(156.06_in, 266.78_in), frc::Translation2d(200.46_in, 316.64_in));
const frc::Rectangle2d ZoneFinder::TRENCH_C = frc::Rectangle2d(frc::Translation2d(445.06_in, 0_in), frc::Translation2d(0_in, 0_in));
const frc::Rectangle2d ZoneFinder::TRENCH_D = frc::Rectangle2d(frc::Translation2d(445.06_in, 0_in), frc::Translation2d(0_in, 0_in));

const frc::Rectangle2d ZoneFinder::BUMP_A = frc::Rectangle2d(frc::Translation2d(156.06_in, 61.86_in), frc::Translation2d(200.46_in, 134.86_in));
const frc::Rectangle2d ZoneFinder::BUMP_B = frc::Rectangle2d(frc::Translation2d(156.06_in, 181.78_in), frc::Translation2d(200.46_in, 254.78_in));
const frc::Rectangle2d ZoneFinder::BUMP_C = frc::Rectangle2d(frc::Translation2d(445.06_in, 0_in), frc::Translation2d(0_in, 0_in));
const frc::Rectangle2d ZoneFinder::BUMP_D = frc::Rectangle2d(frc::Translation2d(445.06_in, 0_in), frc::Translation2d(0_in, 0_in));

ZoneFinder::ZoneFinder(std::shared_ptr<Localizer> _localizer)
{
    frc::Translation2d CurrentTrans = _localizer->getPose().frc::Pose2d::Translation();
};

std::string ZoneFinder::GetZone()
{
if(REDZONE.frc::Rectangle2d::Contains(CurrentTrans))
{
    return "REDZONE";
}
else if(BLUEZONE.frc::Rectangle2d::Contains(CurrentTrans))
{
    return "BLUEZONE";
}
else if(NEUTRALZONE.frc::Rectangle2d::Contains(CurrentTrans))
{
    return "NEUTRALZONE";
}
else 
{
    return "UNKNOWN";
}

if(TRENCH_A.frc::Rectangle2d::Contains(CurrentTrans) || TRENCH_B.frc::Rectangle2d::Contains(CurrentTrans) || TRENCH_C.frc::Rectangle2d::Contains(CurrentTrans) || TRENCH_D.frc::Rectangle2d::Contains(CurrentTrans))
{
    return "TRENCH";
}

if(BUMP_A.frc::Rectangle2d::Contains(CurrentTrans) || BUMP_B.frc::Rectangle2d::Contains(CurrentTrans) || BUMP_C.frc::Rectangle2d::Contains(CurrentTrans) || BUMP_D.frc::Rectangle2d::Contains(CurrentTrans))
{
    return "BUMP";
}
};