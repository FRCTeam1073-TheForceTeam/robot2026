#include <utilities/ShooterTable.h>

const double SPEED_SCALE = 0.96;

ShooterTable::ShooterTable() {
    //Old data, probably can delete this
    // hoodTable.insert(1.00_m, 0.00_rad);
    // hoodTable.insert(1.28_m, 0.00_rad);
    // hoodTable.insert(1.51_m, 0.00_rad);
    // hoodTable.insert(1.75_m, 0.04_rad);
    // hoodTable.insert(2.00_m, 0.08_rad);
    // hoodTable.insert(2.25_m, 0.10_rad);
    // hoodTable.insert(2.50_m, 0.12_rad);
    // hoodTable.insert(2.82_m, 0.14_rad);
    // hoodTable.insert(3.00_m, 0.14_rad);
    // hoodTable.insert(3.60_m, 0.18_rad);
    // hoodTable.insert(4.00_m, 0.20_rad);
    // hoodTable.insert(4.50_m, 0.22_rad);
    // hoodTable.insert(5.47_m, 0.23_rad);

    // flywheelTable.insert(1.00_m, 8.48_mps);
    // flywheelTable.insert(1.28_m, 8.48_mps);
    // flywheelTable.insert(1.51_m, 8.66_mps);
    // flywheelTable.insert(1.75_m, 8.83_mps);
    // flywheelTable.insert(2.00_m, 8.83_mps);
    // flywheelTable.insert(2.25_m, 9.00_mps);
    // flywheelTable.insert(2.50_m, 9.33_mps);
    // flywheelTable.insert(2.82_m, 9.50_mps);
    // flywheelTable.insert(3.00_m, 9.50_mps);
    // flywheelTable.insert(3.60_m, 9.80_mps);
    // flywheelTable.insert(4.00_m, 10.0_mps);
    // flywheelTable.insert(4.50_m, 10.5_mps);
    // flywheelTable.insert(5.47_m, 11.3_mps);

    hoodTable.insert(1.00_m, 0.00_rad);
    hoodTable.insert(1.50_m, 0.062_rad);
    hoodTable.insert(2.00_m, 0.094_rad);
    hoodTable.insert(2.30_m, 0.11_rad);
    hoodTable.insert(2.72_m, 0.12_rad);
    hoodTable.insert(3.00_m, 0.14_rad);
    hoodTable.insert(3.38_m, 0.17_rad);
    hoodTable.insert(3.85_m, 0.20_rad);
    hoodTable.insert(4.20_m, 0.235_rad);
    hoodTable.insert(4.76_m, 0.28_rad);

    flywheelTable.insert(1.00_m, 8.00_mps * SPEED_SCALE);
    flywheelTable.insert(1.50_m, 8.20_mps * SPEED_SCALE);
    flywheelTable.insert(2.00_m, 8.80_mps * SPEED_SCALE);
    flywheelTable.insert(2.30_m, 9.00_mps * SPEED_SCALE);
    flywheelTable.insert(2.72_m, 9.20_mps * SPEED_SCALE);
    flywheelTable.insert(3.00_m, 9.40_mps * SPEED_SCALE);
    flywheelTable.insert(3.38_m, 9.80_mps * SPEED_SCALE);
    flywheelTable.insert(3.85_m, 10.0_mps * SPEED_SCALE);
    flywheelTable.insert(4.20_m, 10.4_mps * SPEED_SCALE);
    flywheelTable.insert(4.76_m, 10.6_mps * SPEED_SCALE);
};

units::angle::radian_t ShooterTable::GetHoodAngle(units::length::meter_t range){
    return hoodTable[range];
};

units::velocity::meters_per_second_t ShooterTable::GetFlywheelVelocity(units::length::meter_t range) {
    return flywheelTable[range];
};
