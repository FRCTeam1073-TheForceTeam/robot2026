// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/ShooterLoad.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
ShooterLoad::ShooterLoad() :
_hardwareConfigured(true),
_loadMotor(LoadMotorId, CANBus("rio")),
_loadVelocitySig(_loadMotor.GetVelocity()),
_loadCurrentSig(_loadMotor.GetTorqueCurrent()),
_commandVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandVelocityVoltage.WithSlot(0);  // Velocity control loop uses these gains.

  _targetVelocity = 0_tps;

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ShooterLoad: Hardware Failed To Configure!" << std::endl;
  }

}


  /// Set the command for the system.
void ShooterLoad::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void ShooterLoad::SetTargetLoadVelocity(units::angular_velocity::turns_per_second_t Velocity) {
  _targetVelocity = Velocity;
}

ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> ShooterLoad::GetLoadVelocity() {
  return _loadVelocitySig;
}

units::angular_velocity::turns_per_second_t ShooterLoad::GetLoadTargetVelocity() {
  return _targetVelocity;
}


void ShooterLoad::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_loadVelocitySig, _loadCurrentSig);

  // Populate feedback cache:
  _feedback.force = _loadCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
  _feedback.velocity = _loadVelocitySig.GetValue() / TurnsPerMeter; // Convert from hardare units to subsystem units.

  _loadMotor.Set(_targetVelocity.value());


  _loadMotor.Set(limiter.Calculate(_targetVelocity).value());
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool ShooterLoad::ConfigureHardware() {
configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot 0 for the velocity control loop:
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.15;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
  

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _loadMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "ShooterLoad: Configuration went wrong" << std::endl;
    }

    // Set our neutral mode to brake on:
    status = _loadMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        std::cerr << "ShooterLoad: Neutral brake went wrong" << std::endl;
    }
    //TODO: change error messages if they are incorrect which they probably are

    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    _loadMotor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return false;

}
