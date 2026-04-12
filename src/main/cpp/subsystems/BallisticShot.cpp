// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/BallisticShot.h"
#include <units/math.h>

BallisticShot::BallisticShot(std::shared_ptr<TargetFinder>& tf) : m_tf(tf) {
    m_currentShot.FlywheelSpeed = 0.0_mps;
    m_currentShot.HoodAngle = 69.2_deg;
    m_currentShot.ShotTime = 2.0_s;
}

BallisticShot::Shot BallisticShot::ComputeShot(units::length::meter_t range){
    Shot shot;
    auto tempHeightAboveHub = heightAboveHub;
    if (range > 3_m) {
        tempHeightAboveHub += (range - 3.0_m) * 0.6;
    }
    // hub is 6 feet tall or 1.829 meters
    auto maxHeight = tempHeightAboveHub + hubHeight - turretHeight;
    auto yVel = units::math::sqrt((maxHeight * 2 * gravity));
    auto timeToMaxHeight = yVel / gravity;
    auto timeToFall = units::math::sqrt((2 * tempHeightAboveHub / gravity));
    shot.ShotTime = timeToMaxHeight + timeToFall;

    auto xVel = range / (shot.ShotTime);

    auto hoodAngle = units::math::atan2(yVel, xVel);

    auto flywheelSpeed = units::math::sqrt((xVel * xVel + yVel * yVel)) / efficiency; //TODO: change efficiency

    shot.FlywheelSpeed = flywheelSpeed;
    shot.HoodAngle = hoodAngle + hood_offset;

    // Special case for our mechanism limits:
    if (range < 1.5_m) {
        shot.FlywheelSpeed *= 0.95; // Downscale speed for very short shots.
    }

    return shot;
    
}

 void BallisticShot::Periodic() {
    auto range = m_tf->getFeedback().rangeToTarget;
    m_currentShot = ComputeShot(range);     // Cache the computed shot value so we don't recompute it too often.
 }
