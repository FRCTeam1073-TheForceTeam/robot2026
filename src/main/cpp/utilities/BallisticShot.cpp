// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/BallisticShot.h"

BallisticShot::BallisticShot() = default;

BallisticShot::Shot BallisticShot::GetShot(units::length::meter_t range, units::length::meter_t heightAboveHub){
    // hub is 6 feet tall or 1.829 meters
    maxHeight = heightAboveHub + hubHeight - turretHeight;
    yVel = std::sqrt(((maxHeight) * 2 * gravity).value()) * 1_mps;
    timeToMaxHeight = yVel / gravity;
    timeToFall = std::sqrt((2 * heightAboveHub / gravity).value()) * 1_s;
    xVel = range / (timeToMaxHeight + timeToFall);
    hoodAngle = atan2(yVel.value(), xVel.value()) * 1_rad;
    flywheelSpeed = std::sqrt((xVel * xVel + yVel * yVel).value()) * 1.0_mps / (0.80); //TODO: change scale factor
    shot.flywheelSpeed = flywheelSpeed;
    shot.hoodAngle = hoodAngle;
    shot.timeToFall = timeToFall;
    shot.timeToMaxHeight = timeToMaxHeight;
    return shot;
}
