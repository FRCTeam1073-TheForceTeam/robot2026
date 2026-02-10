// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/ShooterRotater.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
ShooterRotater::ShooterRotater() :
targetAngle(0_rad),
positionAngle(0_rad),
targetVelocity(0_rad_per_s),
velocity(0_rad_per_s),
_hardwareConfigured(true),
_rotaterMotor(RotaterMotorId, CANBus("rio")),
_rotaterPositionSig(_rotaterMotor.GetPosition()),
_rotaterCurrentSig(_rotaterMotor.GetTorqueCurrent()),
_commandPositionVoltage(units::angle::turn_t(0.0)) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandPositionVoltage.WithSlot(0);  // Position control loop uses these gains.

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
  }

}

  /// Set the command for the system.
void ShooterRotater::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void ShooterRotater::SetTargetAngle(units::angle::radian_t newAngle){
  targetAngle = newAngle;
}

units::angle::radian_t ShooterRotater::GetAngle(){
  return positionAngle;
}

void ShooterRotater::SetTargetVelocity(units::angular_velocity::radians_per_second_t vel){
  targetVelocity = vel;
}

units::angular_velocity::radians_per_second_t ShooterRotater::GetVelocity(){
  return velocity;
}

void ShooterRotater::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_rotaterPositionSig, _rotaterCurrentSig);

  // Latency compensate the feedback when you sample a value and its rate:
  //auto compensatedPos = BaseStatusSignal::GetLatencyCompensatedValue(_rotaterPositionSig, _rotaterVelocitySig);

  // Populate feedback cache:
  _feedback.force = _rotaterCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
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
  if (std::holds_alternative<units::length::meter_t>(_command)) {
      // Send position based command:

      // Convert to hardware units:
      positionAngle = std::get<units::length::meter_t>(_command) * TurnsPerMeter;

      // Send to hardware:
      _rotaterMotor.SetControl(_commandPositionVoltage.WithPosition(positionAngle));
  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _rotaterMotor.SetControl(controls::NeutralOut());
  }
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool ShooterRotater::ConfigureHardware() {
configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

  

    // Slot 0 for position control mode:
    configs.Slot0.kV = 0.12; // Motor constant.
    configs.Slot0.kP = 0.1;
    configs.Slot0.kI = 0.01;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kA = 0.0;

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _rotaterMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "ShooterRotater: Control Failed To Configure!" << std::endl;
    }

    // Set our neutral mode to brake on:
    status = _rotaterMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        std::cerr << "ShooterRotater: Neutral mode brake Failed To Configure!" << std::endl;
    }


    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    _rotaterMotor.SetPosition(units::angle::turn_t(0));

    // Log errors.
    return false;

}
