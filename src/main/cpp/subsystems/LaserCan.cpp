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
laserCAN(28)
{
  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
  }

}


  /// Set the command for the system.
void LaserCan::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void LaserCan::InitializeLaser(){
  laserCAN.set_ranging_mode(grpl::LaserCanRangingMode::Long);//TODO: set ranging mode
  laserCAN.set_timing_budget(grpl::LaserCanTimingBudget::TB100ms);//TODO: set timing budget
  laserCAN.set_roi(grpl::LaserCanROI{8, 8, 16, 16});//TODO: change values
}

void LaserCan::Periodic() {
   std::optional<grpl::LaserCanMeasurement> measurementData = laserCAN.get_measurement();
  if (measurementData.has_value() && measurementData.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT){
    std::cout << "Distance from target:: " << measurementData.value().distance_mm << "millimeters" << std::endl;
  }
  else {
    std::cout << "Error: LaserCAN target is out of range or the measurment has an error" << std::endl;
  }

  frc::SmartDashboard::PutNumber("LaserCan/measurementData", measurementData.value().distance_mm);
  
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool LaserCan::ConfigureHardware() {
configs::TalonFXConfiguration configs{};



//    if (!status.IsOK()) {
        // Log errors.
    //}



    // Log errors.
    return true;

}

