// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Turret.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Turret::Turret() :
velocity(0_rad_per_s),
_hardwareConfigured(true),
_rotaterMotor(RotaterMotorId, CANBus("rio")),
_rotaterPositionSig(_rotaterMotor.GetPosition()),
_rotaterCurrentSig(_rotaterMotor.GetTorqueCurrent()),
_commandPositionVoltage(units::angle::turn_t(0.0)),
_commandVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)) {
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandPositionVoltage.WithSlot(0);  // Position control loop uses these gains.
  _commandVelocityVoltage.WithSlot(1);

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
  }

}

  /// Set the command for the system.
void Turret::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Turret::Periodic() {
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
  if (std::holds_alternative<units::radians_per_second_t>(_command)) {
    auto motorVelocity = std::get<units::radians_per_second_t>(_command) * TurretToMotorTurns;

    _rotaterMotor.SetControl(_commandPositionVoltage.WithVelocity(motorVelocity));
  }
  if (std::holds_alternative<units::radian_t>(_command)) {
      // Send position based command:

      // Convert to hardware units:
      auto motorAngle = std::get<units::radian_t>(_command) * TurretToMotorTurns;

      // Send to hardware:
      _rotaterMotor.SetControl(_commandPositionVoltage.WithPosition(motorAngle));
  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _rotaterMotor.SetControl(controls::NeutralOut());
  }
  frc::SmartDashboard::PutNumber("Turret/Position Rad", _rotaterMotor.GetPosition().GetValue().value() / TurretToMotorTurns * 2 * std::numbers::pi);
  frc::SmartDashboard::PutNumber("Turret/Position Deg", _rotaterMotor.GetPosition().GetValue().value() / TurretToMotorTurns * 360);
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Turret::ConfigureHardware() {
configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

  

    // Slot 0 for position control mode:
    configs.Slot0.kV = 0.153; // Motor constant.
    configs.Slot0.kP = 0.4;
    configs.Slot0.kI = 0.1;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.1;

    // Slot 1 is velocity
    configs.Slot1.kV = 0.153;
    configs.Slot1.kP = 0.1;
    configs.Slot1.kI = 0.0;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _rotaterMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "Turret: Control Failed To Configure!" << std::endl;
    }

    // Set our neutral mode to brake on:
    status = _rotaterMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        std::cerr << "Turret: Neutral mode brake Failed To Configure!" << std::endl;
    }


    // Depends on mechanism/subsystem design:
    // Optionally start out at zero after initialization:
    // _rotaterMotor.SetPosition(units::angle::degree_t(95.0) * TurretToMotorTurns);
    _rotaterMotor.SetPosition(0_deg);

    // Log errors.
    return false;

}
