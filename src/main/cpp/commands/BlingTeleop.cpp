// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BlingTeleop.h"


using namespace ctre::phoenix6;

BlingTeleop::BlingTeleop(std::shared_ptr<Bling>& Bling, std::shared_ptr<OI>& OI) :
  m_bling(Bling),
  m_OI(OI) {
    AddRequirements({m_bling.get()});
  
}

// Called when the command is initially scheduled.
void BlingTeleop::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void BlingTeleop::Execute() {
  controls::SolidColor color(8, 20);
  _currentVoltage = frc::RobotController::GetBatteryVoltage() * alpha + (1.0 - alpha) * _currentVoltage;
  if (_currentVoltage >= 12.4_V) {
    color.WithColor(signals::RGBWColor (0, 0, 127)); //Color is dimmer to conserve battery
  } else if (_currentVoltage >= 12.1_V) {
    color.WithColor(signals::RGBWColor (127, 0, 82));
  } else {
    color.WithColor(signals::RGBWColor (127, 0, 0));
  }

  m_bling->SetCommand(color);

  // code for changing bling color based on proximity to dead spot on turret
  // controls::SolidColor color(0, 8);
  // _currentVoltage = frc::RobotController::GetBatteryVoltage() * alpha + (1.0 - alpha) * _currentVoltage;
  // if ([get the turret angle feedback] <= -150_deg or [get the turret angle feedback] >= 141_deg) {
  //   color.WithColor(signals::RGBWColor (127, 0, 0)); //Color is dimmer to conserve battery
  // } else if ([get the turret angle feedback] <= -90_deg or [get the turret angle feedback] >= 78_deg) {
  //   color.WithColor(signals::RGBWColor (82, 0, 127));
  // } else {
  //   color.WithColor(signals::RGBWColor (0, 0, 127));
  // }

  // m_bling->SetCommand(color);
}

// Called once the command ends or is interrupted.
void BlingTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool BlingTeleop::IsFinished() {
  return false;
}
