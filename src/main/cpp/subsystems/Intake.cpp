// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
#include <iostream>

#include "subsystems/Intake.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Intake::Intake() :
_hardwareConfigured(false),
_intakeMotor(IntakeMotorId, CANBus("rio")),
_intakePositionSig(_intakeMotor.GetPosition()),
_intakeCurrentSig(_intakeMotor.GetTorqueCurrent()),
_commandPositionVoltage(units::angle::turn_t(0.0)),
_command(0.0_m),
_limiter(10.0_mps/1.0_s) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandPositionVoltage.WithSlot(0);  // Velocity control loop uses these gains.

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "Spindexer: Hardware Failed To Configure!" << std::endl;
  }

  frc::SmartDashboard::PutBoolean("Spindexer/Spindexer - hardware_configured", _hardwareConfigured);
}

// Set the command for the system.
void Intake::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}


void Intake::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_intakePositionSig, _intakeCurrentSig);

  // Populate feedback cache:
  _feedback.force = _intakeCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
  _feedback.position = _intakePositionSig.GetValue() / TurnsPerMeter; // Convert from hardware units to subsystem units. 

  // // Process command:
  if (std::holds_alternative<units::length::meter_t>(_command)) {
      // Send position based command:
      auto MotorPosition = std::get<units::length::meter_t>(_command) * TurnsPerMeter;
      // Send to hardware:
      _intakeMotor.SetControl(_commandPositionVoltage.WithPosition(MotorPosition));

  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _intakeMotor.SetControl(controls::NeutralOut());
    _limiter.Reset(0.0_mps);
  }

  frc::SmartDashboard::PutNumber("Intake/Position(mps)", _feedback.velocity.value());  
  frc::SmartDashboard::PutNumber("Intake/TargetPosition(mps)", _limiter.LastValue().value());  
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Intake::ConfigureHardware() {
  configs::TalonFXConfiguration configs{};

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

  configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
  configs.Voltage.PeakReverseVoltage = -8_V;

  // Slot 0 for the velocity control loop:
  configs.Slot0.kV = 0.12;
  configs.Slot0.kP = 0.35;
  configs.Slot0.kI = 0.0;
  configs.Slot0.kD = 0.0;
  configs.Slot0.kA = 0.0;
  configs.Slot0.kS = 0.04;

  
  // Set whether motor control direction is inverted or not:
  configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

  // Set the control configuration for the drive motor:
  auto status = _intakeMotor.GetConfigurator().Apply(configs, 1_s); // 1 Second configuration timeout.

  if (!status.IsOK()) {
      std::cerr << "Intake: config failed to config!" << std::endl;
      return false;
  }

  // Set our neutral mode to brake on:
  status = _intakeMotor.SetNeutralMode(signals::NeutralModeValue::Coast, 1_s);

  if (!status.IsOK()) {
      std::cerr << "Intake: neutral mode failed to config :(!" << std::endl;
      return false;
  }

  // Log errors.
  return true;
}
