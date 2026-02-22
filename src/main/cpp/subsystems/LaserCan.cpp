// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "subsystems/LaserCan.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
LaserCan::LaserCan() :
_hardwareConfigured(true), 
_laserCAN(28)
{
  _feedback.is_valid = false;
  _feedback.range = 0.0_m;

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "LaserCAN: Hardware Failed To Configure!" << std::endl;
  }

  frc::SmartDashboard::PutNumber("LaserCan/hardware_configured", _hardwareConfigured);
}


void LaserCan::Periodic() {
  auto measurementData = _laserCAN.get_measurement();
  if (measurementData.has_value() && measurementData.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    _feedback.is_valid = true;
    _feedback.range = units::length::millimeter_t(measurementData.value().distance_mm);
  }
  else {
    _feedback.is_valid = false;
    _feedback.range = 0.0_m;
  }

  frc::SmartDashboard::PutNumber("LaserCan/is_valid", _feedback.is_valid);
  frc::SmartDashboard::PutNumber("LaserCan/distance(m)", _feedback.range.value());
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool LaserCan::ConfigureHardware() {

  _laserCAN.set_ranging_mode(grpl::LaserCanRangingMode::Long);//TODO: set ranging mode
  _laserCAN.set_timing_budget(grpl::LaserCanTimingBudget::TB100ms);//TODO: set timing budget
  _laserCAN.set_roi(grpl::LaserCanROI{8, 8, 16, 16});//TODO: change values

  return true;
}

