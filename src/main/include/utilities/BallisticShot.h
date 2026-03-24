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
    units::time::second_t ShotTime;
  };

  static Shot GetShot(units::length::meter_t range, units::length::meter_t heightAboveHub);
  
  BallisticShot();
 private:
  static constexpr units::length::meter_t hubHeight = 1.829_m;
  static constexpr units::length::meter_t turretHeight = 0.40_m;
  static constexpr double efficiency = 0.75;   // Calibration for transfer of flywheel velocity to fuel.
  static constexpr units::angle::radian_t hood_offset = -0.2_rad; // Calibration for shot exit vs. hood angle.

  static constexpr units::acceleration::meters_per_second_squared_t gravity = 9.8_mps_sq;

};
