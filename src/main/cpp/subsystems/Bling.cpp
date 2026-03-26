// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Bling.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Bling::Bling() :
  _hardwareConfigured(true),
  _CANdle(CANdleId, CANBus::RoboRIO()) // Might not be roboRio figure out what it is if not
   {
  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
  }
}


void Bling::Periodic() {
      // // Process command:
  if (std::holds_alternative<int>(_command)) {
    
  }
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Bling::ConfigureHardware() {
configs::TalonFXConfiguration configs{};
    // Log errors.
    return false;
}
