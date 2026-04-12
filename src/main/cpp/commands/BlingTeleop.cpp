// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BlingTeleop.h"


using namespace ctre::phoenix6;

BlingTeleop::BlingTeleop(std::shared_ptr<Bling>& Bling, std::shared_ptr<OI>& OI) :
  m_bling(Bling),
  m_OI(OI),
  _batteryVoltage(0.0_V) {
    AddRequirements({m_bling.get()});
  
}

// Called when the command is initially scheduled.
void BlingTeleop::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void BlingTeleop::Execute() {
  // Set LEDs on the CANdle itself for battery...
  controls::SolidColor color(0, 20);
  _batteryVoltage = frc::RobotController::GetBatteryVoltage() * alpha + (1.0 - alpha) * _batteryVoltage; // Filtered battery vol;tage.
  if (_batteryVoltage >= 12.4_V) {
    color.WithColor(signals::RGBWColor (0, 0, 127)); // Color is dimmer to conserve battery
  } else if (_batteryVoltage >= 12.1_V) {
    color.WithColor(signals::RGBWColor (127, 0, 82));
  } else {
    // Scaled voltage dimming as we die...
    auto delta = (12.1_V - _batteryVoltage).value() / 5.0; // Minimum is about 7V so range is about 5V scale it from ~  (right at 12.1) to 1 (down at 7 almost dead)
    int red_color = 127 - 100 * delta;
    color.WithColor(signals::RGBWColor (red_color, 0, 0));
  }

  m_bling->SetCommand(color);
}

// Called once the command ends or is interrupted.
void BlingTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool BlingTeleop::IsFinished() {
  return false;
}
