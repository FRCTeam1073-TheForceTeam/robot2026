// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/BallisticShot.h"
#include <units/math.h>

BallisticShot::BallisticShot() = default;

BallisticShot::Shot BallisticShot::GetShot(units::length::meter_t range){
    Shot shot;
    // hub is 6 feet tall or 1.829 meters
    auto maxHeight = heightAboveHub + hubHeight - turretHeight;
    auto yVel = units::math::sqrt((maxHeight * 2 * gravity));
    auto timeToMaxHeight = yVel / gravity;
    auto timeToFall = units::math::sqrt((2 * heightAboveHub / gravity));
    shot.ShotTime = timeToMaxHeight + timeToFall;

    auto xVel = range / (shot.ShotTime);

    auto hoodAngle = units::math::atan2(yVel, xVel);

    auto flywheelSpeed = units::math::sqrt((xVel * xVel + yVel * yVel)) / efficiency; //TODO: change efficiency

    shot.FlywheelSpeed = flywheelSpeed;
    shot.HoodAngle = hoodAngle + hood_offset;
    return shot;
    
}
