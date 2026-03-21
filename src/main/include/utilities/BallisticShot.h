// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include<units/velocity.h>
#include<units/angle.h>
#include<units/length.h>
#include<units/acceleration.h>
#include<math.h>

class BallisticShot {
 public:
  struct Shot
  {
    units::velocity::meters_per_second_t FlywheelSpeed;
    units::angle::radian_t HoodAngle;
  };

  Shot GetShot(units::length::meter_t range, units::length::meter_t heightAboveHub){}
  
  BallisticShot();
 private:
  units::length::meter_t maxHeight;
  const units::length::meter_t hubHeight = 1.829_m;
  const units::length::meter_t turretHeight = 0.41_m;

  const units::acceleration::meters_per_second_squared_t gravity = 9.8_mps_sq;

  units::velocity::meters_per_second_t xVel;
  units::velocity::meters_per_second_t yVel;
  units::time::second_t timeToMaxHeight;
  units::time::second_t timeToFall;

  units::velocity::meters_per_second_t flywheelSpeed;
  units::angle::radian_t hoodAngle;

  Shot shot;

};
