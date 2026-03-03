#pragma once
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <wpi/interpolating_map.h>

class ShooterTable{
    public:
        ShooterTable();

        units::angle::radian_t GetHoodAngle(units::length::meter_t range);

        units::velocity::meters_per_second_t GetFlywheelVelocity(units::length::meter_t range);

    private:
        wpi::interpolating_map<units::length::meter_t, units::angle::radian_t> hoodTable;
        wpi::interpolating_map<units::length::meter_t, units::velocity::meters_per_second_t> flywheelTable;
};

