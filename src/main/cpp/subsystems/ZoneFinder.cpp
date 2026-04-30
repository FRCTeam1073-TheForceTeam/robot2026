#include "subsystems/ZoneFinder.h"


ZoneFinder::ZoneFinder(std::shared_ptr<Localizer>& localizer) : _localizer(localizer)
{
    //AndyMark Dimentions
    // zones.push_back(Zone("BLUEZONE", frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(181.56_in, 316.64_in))));
    // zones.push_back(Zone("REDZONE", frc::Rectangle2d(frc::Translation2d(468.56_in, 0_in), frc::Translation2d(650.12_in, 316.64_in))));
    // zones.push_back(Zone("NEUTRALZONE", frc::Rectangle2d(frc::Translation2d(181.56_in, 0_in), frc::Translation2d(468.56_in, 316.64_in))));
    
    // //Blue alliance POV
    // zones.push_back(Zone("RIGHTHALF", frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(650.12_in, 158.32_in))));
    // zones.push_back(Zone("LEFTHALF", frc::Rectangle2d(frc::Translation2d(0_in, 158.32_in), frc::Translation2d(650.12_in, 316.64_in))));
    

    // //Previously: added 35 in. (robot dimentions w/ bumpers) in x-dimention on either side to expand trench zones. -35 to first x-value, +35 to second x-value
    // zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(139.06_in, 0_in), frc::Translation2d(229.46_in, 49.86_in))));
    // zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(139.06_in, 266.78_in), frc::Translation2d(229.46_in, 316.64_in))));
    // zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(416.06_in, 0_in), frc::Translation2d(506.46_in, 49.86_in))));
    // zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(416.06_in, 266.78_in), frc::Translation2d(506.46_in, 316.64_in))));
    
    // zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(156.06_in, 61.86_in), frc::Translation2d(200.46_in, 134.86_in))));
    // zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(156.06_in, 181.78_in), frc::Translation2d(200.46_in, 254.78_in))));
    // zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(445.06_in, 61.86_in), frc::Translation2d(489.46_in, 134.86_in))));
    // zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(445.06_in, 181.78_in), frc::Translation2d(489.46_in, 254.78_in))));

    //Welded Dimentions
    zones.push_back(Zone("BLUEZONE", frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(156.61_in, 317.69_in))));
    zones.push_back(Zone("REDZONE", frc::Rectangle2d(frc::Translation2d(490.01_in, 0_in), frc::Translation2d(651.22_in, 317.69_in))));
    zones.push_back(Zone("NEUTRALZONE", frc::Rectangle2d(frc::Translation2d(201.01_in, 0_in), frc::Translation2d(325.61_in, 317.69_in))));
    
    //Blue alliance POV
    zones.push_back(Zone("RIGHTHALF", frc::Rectangle2d(frc::Translation2d(0_in, 0_in), frc::Translation2d(651.22_in, 158.84_in))));
    zones.push_back(Zone("LEFTHALF", frc::Rectangle2d(frc::Translation2d(0_in, 158.84_in), frc::Translation2d(651.22_in, 317.69_in))));
    
    //added 35 in. (robot dimentions w/ bumpers) in x-dimention on either side to expand trench zones. -35 to first x-value, +35 to second x-value
    zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(121.61_in, 0_in), frc::Translation2d(236.01_in, 50.35_in))));
    zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(121.61_in, 267.098_in), frc::Translation2d(236.01_in, 317.69_in))));
    zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(410.61_in, 0_in), frc::Translation2d(525.01_in, 50.35_in))));
    zones.push_back(Zone("TRENCH", frc::Rectangle2d(frc::Translation2d(410.61_in, 267.098_in), frc::Translation2d(525.01_in, 317.69_in))));
    
    zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(156.61_in, 62.59_in), frc::Translation2d(201.01_in, 135.59_in))));
    zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(156.61_in, 182.1_in), frc::Translation2d(201.01_in, 255.1_in))));
    zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(445.61_in, 62.59_in), frc::Translation2d(490.01_in, 135.59_in))));
    zones.push_back(Zone("BUMP", frc::Rectangle2d(frc::Translation2d(445.61_in, 182.1_in), frc::Translation2d(490.01_in, 255.1_in))));
};

std::set<std::string> ZoneFinder::GetZones()
{
    std::set<std::string> result;
    for(const auto &zone : zones)
    {
        if(zone.rect.Contains(CurrentTrans))
        {
            result.insert(zone.name);
        }
    }
    return result;
}


void ZoneFinder::Periodic()
{
    _alliance = frc::DriverStation::GetAlliance();
    CurrentTrans = _localizer->getPose().Translation();
    auto result = GetZones();


    _alliance = frc::DriverStation::GetAlliance();
    std::string zonelist;
    for(const auto &zone : result)
    {
        std::string zonePortion = zone;
        //this part switches left and right for red alliance POV since all code is from blue alliance POV
        if(_alliance.value() == frc::DriverStation::Alliance::kRed && zone.find("RIGHTHALF") != std::string::npos)
        {
            zonePortion = "LEFTHALF";
        }
        else if(_alliance.value() == frc::DriverStation::Alliance::kRed && zone.find("LEFTHALF") != std::string::npos)
        {
            zonePortion = "RIGHTHALF";
        }
        zonelist += zonePortion + ", ";
    }
    frc::SmartDashboard::PutString("Zone/Zone", zonelist);
}