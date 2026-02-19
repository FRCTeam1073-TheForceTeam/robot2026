// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/ShooterHood.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
ShooterHood::ShooterHood() :
_hardwareConfigured(true),
_hoodMotor(HoodMotorId, CANBus("rio")),
_hoodPositionSig(_hoodMotor.GetPosition()),
_hoodCurrentSig(_hoodMotor.GetTorqueCurrent()),
_commandPositionVoltage(units::angle::turn_t(0.0)),
_commandVelocityVoltage(units::angular_velocity::radians_per_second_t(0.0)) {
  // Extra implementation of subsystem constructor goes here.
  TargetPosition = 0_rad;
  Position = 0_rad;

  // Assign gain slots for the commands to use:
  _commandPositionVoltage.WithSlot(1);  // Position control loop uses these gains.
  _commandVelocityVoltage.WithSlot(0);

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
  }

}


  /// Set the command for the system.
void ShooterHood::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void ShooterHood::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_hoodPositionSig, _hoodCurrentSig);

  // Latency compensate the feedback when you sample a value and its rate:
  //auto compensatedPos = BaseStatusSignal::GetLatencyCompensatedValue(_hoodPositionSig, _hoodVelocitySig);

  // Populate feedback cache:
  _feedback.current = _hoodCurrentSig.GetValue(); // Convert from hardware units to subsystem units.
  //_feedback.position = compensatedPos / TurnsPerMeter; // Convert from hardare units to subsystem units. Divide by conversion to produce feedback.
  //_feedback.velocity = _exampleVelocitySig.GetValue() / TurnsPerMeter; // Convert from hardare units to subsystem units.


  // // Process command:
  // if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
  //     // Send velocity based command:

  //     // Convert to hardware units:
  //     // Multiply by conversion to produce commands.
  //     auto angular_vel = std::get<units::velocity::meters_per_second_t>(_command) * TurnsPerMeter;
  //     // Send to hardware:
  //     _exampleMotor.SetControl(_commandVelocityVoltage.WithVelocity(angular_vel));
  if (std::holds_alternative<units::angular_velocity::radians_per_second_t>(_command)) {
      // Send position based command:

      // Convert to hardware units:
      auto motorAngularVelocity = std::get<units::angular_velocity::radians_per_second_t>(_command) * HoodToMotorGearRatio;

      // Send to hardware:
      _hoodMotor.SetControl(_commandVelocityVoltage.WithVelocity(motorAngularVelocity));
  } 
  else if (std::holds_alternative<units::angle::radian_t>(_command)) {
      // Send position based command:

      // Convert to hardware units:
      auto motorAngle = std::get<units::angle::radian_t>(_command) * HoodToMotorGearRatio;

      // Send to hardware:
      _hoodMotor.SetControl(_commandPositionVoltage.WithPosition(motorAngle));
  }
  else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _hoodMotor.SetControl(controls::NeutralOut());
  }

  frc::SmartDashboard::PutNumber("Hood Angle Position", Position.value() / HoodToMotorGearRatio);

}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool ShooterHood::ConfigureHardware() {
configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

    

    // Slot 0 for velocity control mode:
    configs.Slot0.kV = 0.153; // Motor constant.
    configs.Slot0.kP = 0.15;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;

    // Slot 0 for position control mode:
    configs.Slot1.kV = 0.153; // Motor constant.
    configs.Slot1.kP = 0.11;
    configs.Slot1.kI = 0.02;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _hoodMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "ShooterHood: Control Failed To Configure!" << std::endl;

    }

    // Set our neutral mode to brake on:
    status = _hoodMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        std::cerr << "ShooterHood: Neutral mode brake Failed To Configure!" << std::endl;
    }


    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    _hoodMotor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return false;

}
