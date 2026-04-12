// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Bling.h"

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
      // // Process command:
   if (std::holds_alternative<controls::SolidColor>(_command)) {
      auto color = std::get<controls::SolidColor>(_command);
      _CANdle.SetControl(color);
   }
   else {
    _CANdle.SetControl(controls::SolidColor(8, 20).WithColor(signals::RGBWColor (255, 255, 255)));
   }
}

frc2::CommandPtr Bling::blingWhite() {
  return RunOnce([this] {SetCommand(controls::SolidColor(8, 20).WithColor(signals::RGBWColor (255, 255, 255)));});
}

//TODO: change these commands to on and off

frc2::CommandPtr Bling::blingPurple() {
  return RunOnce([this] {SetCommand(controls::SolidColor(8, 20).WithColor(signals::RGBWColor (147, 112, 219)));});
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Bling::ConfigureHardware() {
  //_CANdle.ClearAllAnimations();
  //controls::SolidColor color(0, 7);
  //color.WithColor(signals::RGBWColor (0, 255, 0));
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
