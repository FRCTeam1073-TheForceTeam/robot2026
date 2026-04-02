// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Bling.h"

#include <frc/RobotController.h>


using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Bling::Bling() :
  _hardwareConfigured(true),
  _currentVoltage(0_V),
  _CANdle(CANdleId, CANBus::RoboRIO()) // Might not be roboRio figure out what it is if not
   {
  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "CANdle: Hardware Failed To Configure!" << std::endl;
  }
}

void Bling::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Bling::Periodic() {
  controls::SolidColor batColor(0, 7);// CANdle will display the battery status
  controls::SolidColor neutralColor1(8, 42); //color that will display when the turret is not pointed in this direction
  controls::SolidColor neutralColor2(45, 79); //Change the value of this color to save battery; it should be a dull but visible grey
  //43-44 is the center of the front of the robot 
  

      // // Process command:
   if (std::holds_alternative<std::string>(_command)) {
  //   if (std::get<std::string>(_command) == "red") {
  //     controls::SolidColor batColor(0, 7);
  //     batColor.WithColor(signals::RGBWColor (255, 0, 0));
  //     _CANdle.SetControl (batColor);
  //   }
  //   if (std::get<std::string>(_command) == "green") {
  //     controls::SolidColor batColor(0, 7);
  //     batColor.WithColor(signals::RGBWColor (0, 255, 0));
  //     _CANdle.SetControl (batColor);
  //   }
      if (std::get<std::string>(_command) == "battery") {
          _currentVoltage = frc::RobotController::GetBatteryVoltage() * alpha + (1.0 - alpha) * _currentVoltage;
          if (_currentVoltage >= 12.4_V) {
            batColor.WithColor(signals::RGBWColor (0, 0, 127)); //Color is dimmer to conserve battery
          } else if (_currentVoltage >= 12.1_V) {
            batColor.WithColor(signals::RGBWColor (127, 0, 82));
          } else {
            batColor.WithColor(signals::RGBWColor (127, 0, 0));
          }
      }
   }
   _CANdle.SetControl(batColor);
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Bling::ConfigureHardware() {
  //_CANdle.ClearAllAnimations();
  //controls::SolidColor batColor(0, 7);
  //batColor.WithColor(signals::RGBWColor (0, 255, 0));
  //controls::ColorFlowAnimation flow(0, 7);
  //flow.WithColor(signals::RGBWColor (255, 0, 255));
  //controls::FireAnimation burnt(0, 7);
  //controls::RainbowAnimation Rain(0, 7);
  // controls::LarsonAnimation boing(0, 7);
  // boing.WithColor(signals::RGBWColor (255, 255, 0));
  // controls::TwinkleAnimation star(0,7);
  // star.WithColor(signals::RGBWColor (255, 0, 0));
  // _CANdle.SetControl (star);
    // Log errors.
    return true;
}
