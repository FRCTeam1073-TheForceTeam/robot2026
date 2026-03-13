// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
#include <iostream>

#include "subsystems/Intake.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include "commands/IntakeTeleop.h"

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Intake::Intake() :
_hardwareConfigured(false),
_leadMotor(IntakeLeadId, CANBus("rio")),
_followMotor(IntakeFollowId, CANBus("rio")),
_intakePositionSig(_leadMotor.GetPosition()),
_intakeVelocitySig(_leadMotor.GetVelocity()),
_intakeCurrentSig(_leadMotor.GetTorqueCurrent()),
_commandPositionVoltage(units::angle::turn_t(0.0)),
_commandVelocityVoltage(units::angular_velocity::radians_per_second_t(0.0)),
_command(std::monostate()),
_hasZero(false),
_limiter(7.0_rad_per_s) { //was 5 radians before
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandPositionVoltage.WithSlot(0);  // Position control loop uses slot 0
  _commandVelocityVoltage.WithSlot(1);  // Velocity control loop uses slot 1

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "Intake: Hardware Failed To Configure!" << std::endl;
  }

  frc::SmartDashboard::PutBoolean("Intake/Intake - hardware_configured", _hardwareConfigured);
}

// Set the command for the system.
void Intake::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Intake::Zero() {
  _leadMotor.SetPosition(units::angle::turn_t(-122_deg)*GearRatio); 
  _hasZero = true;
}

void Intake::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_intakePositionSig, _intakeCurrentSig);

  units::angle::radian_t targetAngle(0_rad);
  // Populate feedback cache:
  _feedback.torque = _intakeCurrentSig.GetValue() / AmpsPerNewtonMeter; // Convert from hardware units to subsystem units.
  _feedback.position = _intakePositionSig.GetValue() / GearRatio; // Convert from hardware units to subsystem units.
  _feedback.velocity = _intakeVelocitySig.GetValue() / GearRatio;
  _feedback.hasZero = _hasZero; // Index/Zero reporting.

  // // Process command:
  if (std::holds_alternative<units::angular_velocity::radians_per_second_t>(_command)) {
      units::angular_velocity::turns_per_second_t motorVelocity = std::get<units::angular_velocity::radians_per_second_t>(_command) * GearRatio;
      _leadMotor.SetControl(_commandVelocityVoltage.WithVelocity(motorVelocity));
      _followMotor.SetControl(controls::Follower(_leadMotor.GetDeviceID(), signals::MotorAlignmentValue::Opposed));
      _limiter.Reset(_feedback.position); // Keep the limiter in sync in the other control mode.
  } else if (std::holds_alternative<units::angle::radian_t>(_command)) {
      // auto limitedIntakeTarget = _limiter.Calculate(std::get<units::radian_t>(_command));
      auto clamped_command = clamp(std::get<units::angle::radian_t>(_command), minPosition, maxPosition);
      targetAngle = _limiter.Calculate(clamped_command);

      auto limitedIntakeTarget = _limiter.Calculate(std::get<units::angle::radian_t>(_command));

      // Send position based command:
      units::angle::turn_t motorPosition = limitedIntakeTarget * GearRatio;

      // Send to hardware:
      _leadMotor.SetControl(_commandPositionVoltage.WithPosition(motorPosition));
      _followMotor.SetControl(controls::Follower(_leadMotor.GetDeviceID(), signals::MotorAlignmentValue::Opposed));

  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _leadMotor.SetControl(controls::NeutralOut());
    _followMotor.SetControl(controls::NeutralOut());
    _limiter.Reset(_feedback.position); // Keep the limiter in sync in other control mode.
  }

  frc::SmartDashboard::PutNumber("Intake/Position(rad)", _feedback.position.value());  
  frc::SmartDashboard::PutNumber("Intake/TargetPosition(rad)", _limiter.LastValue().value());  
  frc::SmartDashboard::PutNumber("Intake/Torque(Nm)", _feedback.torque.value());
}

frc2::CommandPtr Intake::IntakeOut() {
  return RunOnce([this] {SetCommand(-0.1_deg);});
}

frc2::CommandPtr Intake::IntakeIn() {
  return RunOnce([this] {SetCommand(-122.0_deg);});
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Intake::ConfigureHardware() {
  configs::TalonFXConfiguration configs{};

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

  configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
  configs.Voltage.PeakReverseVoltage = -8_V;

  // Slot 0 for the position control loop:
  configs.Slot0.kV = 0.153;
  configs.Slot0.kP = 0.4;
  configs.Slot0.kI = 0.02;
  configs.Slot0.kD = 0.01;
  configs.Slot0.kA = 0.0;
  configs.Slot0.kS = 0.02;
  // configs.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;
  // configs.Slot0.kG = 0.0;  // Gravity compensation.

  // Slot 1 for the velocity control loop:
  configs.Slot1.kV = 0.153;
  configs.Slot1.kP = 0.3;
  configs.Slot1.kI = 0.0;
  configs.Slot1.kD = 0.0;
  configs.Slot1.kA = 0.0;
  configs.Slot1.kS = 0.02;
  // configs.Slot1.GravityType = signals::GravityTypeValue::Arm_Cosine;
  // configs.Slot1.kG = 0.0; // Gravity compensation.
  
  // Set whether motor control direction is inverted or not:
  configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

  // Set the control configuration for the drive motor:
  auto status = _leadMotor.GetConfigurator().Apply(configs, 1_s); // 1 Second configuration timeout.

  if (!status.IsOK()) {
      std::cerr << "Intake: leader failed to config!" << std::endl;
      return false;
  }

  // Control mode set in periodic using "Oppopsed" because motor turns opposite of leader.
  configs::TalonFXConfiguration followConfigs{};
  followConfigs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive);

  status = _followMotor.GetConfigurator().Apply(configs, 1_s); // 1 Second configuration timeout.

  if (!status.IsOK()) {
      std::cerr << "Intake: follower failed to config!" << std::endl;
      return false;
  }

  // Set our neutral mode to brake on:
  status = _leadMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

  if (!status.IsOK()) {
      std::cerr << "Intake: neutral mode failed to config :(!" << std::endl;
      return false;
  }

  // Initialize at the zero position:
  _leadMotor.SetPosition(units::angle::turn_t(-122_deg)*GearRatio); 

  // Log errors.
  return true;
}
//yay