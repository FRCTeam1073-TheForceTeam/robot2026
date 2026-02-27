// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Kicker.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Kicker::Kicker() :
_kickerMotor(LoadMotorId, CANBus("rio")),
_kickerVelocitySig(_kickerMotor.GetVelocity()),
_kickerCurrentSig(_kickerMotor.GetTorqueCurrent()),
_commandVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
_command(0.0_mps),
_limiter(10.0_mps/1.0_s),
_hardwareConfigured(true) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandVelocityVoltage.WithSlot(0);  // Velocity control loop uses these gains.

  // _targetVelocity = 0_tps;

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ShooterLoad: Hardware Failed To Configure!" << std::endl;
  }

  frc::SmartDashboard::PutBoolean("Kicker/Kicker - hardware_configured", _hardwareConfigured);
}

void Kicker::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Kicker::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_kickerVelocitySig, _kickerCurrentSig);

  // Populate feedback cache:
  _feedback.force = _kickerCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
  _feedback.velocity = _kickerVelocitySig.GetValue() / (TurnsPerMeter * GearRatio); // Convert from hardare units to subsystem units.

  if (std::holds_alternative<units::meters_per_second_t>(_command)) {
    auto limitedVelocity = _limiter.Calculate(std::get<units::meters_per_second_t>(_command));
    auto motorVelocity = limitedVelocity * TurnsPerMeter * GearRatio;
    
    _kickerMotor.SetControl(_commandVelocityVoltage.WithVelocity(motorVelocity));
  } else {
    _kickerMotor.SetControl(controls::NeutralOut());
    _limiter.Reset(0.0_mps);
  }

  frc::SmartDashboard::PutNumber("Kicker/Velocity(mps)", _feedback.velocity.value());
  frc::SmartDashboard::PutNumber("Kicker/TargetVelocity(mps)", _limiter.LastValue().value());
}

frc2::CommandPtr Kicker::SpinToSpeed(units::meters_per_second_t velocity) {
  return RunOnce([this, velocity] {SetCommand(velocity);});
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Kicker::ConfigureHardware() {
configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot 0 for the velocity control loop:
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.25;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.04;
  

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _kickerMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "ShooterLoad: Configuration went wrong" << std::endl;
        return false;
    }

    // Set our neutral mode to brake on:
    status = _kickerMotor.SetNeutralMode(signals::NeutralModeValue::Coast, 1_s);

    if (!status.IsOK()) {
        std::cerr << "ShooterLoad: Neutral brake went wrong" << std::endl;
        return false;
    }

    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    _kickerMotor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return true;
}
