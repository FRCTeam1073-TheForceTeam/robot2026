// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
#include <iostream>

#include "subsystems/Climber.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

/**
 * You have to use initializer lists to build up the elements of the subsystem in the right order.
 */
Climber::Climber() :
_hardwareConfigured(false),
_climberMotor(ClimberMotorId, CANBus("Canivore")),
_climberVelocitySig(_climberMotor.GetVelocity()),
_climberCurrentSig(_climberMotor.GetTorqueCurrent()),
_climberPositionSig(_climberMotor.GetPosition()),
_commandVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
_commandPositionVoltage(units::angle::turn_t(0.0)),
_command(0.0_mps),
_limiter(10.0_mps/1.0_s),
_positionLimiter(0.104_m/1.0_s)
{
  // Extra implementation of subsystem constructor goes here.

  // Assign gain slots for the commands to use:
  _commandVelocityVoltage.WithSlot(0);  // Velocity control loop uses these gains.
  _commandPositionVoltage.WithSlot(1); 

  // Do hardware configuration and track if it succeeds:
  _hardwareConfigured = ConfigureHardware();
  if (!_hardwareConfigured) {
    std::cerr << "Climber: Hardware Failed To Configure!" << std::endl;
  }

  frc::SmartDashboard::PutBoolean("Climber/Climber - hardware_configured", _hardwareConfigured);
}

// Set the command for the system.
void Climber::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Climber::Zero() {
  _climberMotor.SetPosition(units::angle::turn_t(0));
}

bool Climber::IsHooked() {
  return m_ClimberOnInput.Get();
}


units::meter_t Climber::getClimberPosition() {
  return _feedback.position;
}

void Climber::Periodic() {
  // Sample the hardware:
  BaseStatusSignal::RefreshAll(_climberVelocitySig, _climberCurrentSig, _climberPositionSig);

  units::length::meter_t climberPosition(0_m);

  // Populate feedback cache:
  _feedback.force = _climberCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.
  _feedback.velocity = _climberVelocitySig.GetValue() / (TurnsPerMeter * GearRatio); // Convert from hardware units to subsystem units. 
  _feedback.position = _climberPositionSig.GetValue() / (TurnsPerMeter * GearRatio);

  // // Process command:
  if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
      // Send position based command:
    auto limitedVel = _limiter.Calculate(std::get<units::velocity::meters_per_second_t>(_command) );
      // Send to hardware:
    auto motorVel = limitedVel * TurnsPerMeter * GearRatio;

    _climberMotor.SetControl(_commandVelocityVoltage.WithVelocity(motorVel));



  } else if (std::holds_alternative<units::length::meter_t>(_command)) {
    auto limitedPos = _positionLimiter.Calculate(std::get<units::length::meter_t>(_command));
    auto clamped_command = clamp(limitedPos, minPosition, maxPosition);

    // climberPosition = _positionLimiter.Calculate(clamped_command);

    auto motorPosition = clamped_command * TurnsPerMeter * GearRatio;

    _climberMotor.SetControl(_commandPositionVoltage.WithPosition(motorPosition));

    frc::SmartDashboard::PutNumber("Climber/TargetPostion", clamped_command.value());

  }
    else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _climberMotor.SetControl(controls::NeutralOut());
    _limiter.Reset(0.0_mps);
  }

  frc::SmartDashboard::PutNumber("Climber/Velocity(mps)", _feedback.velocity.value());  
  frc::SmartDashboard::PutNumber("Climber/TargetVelocity(mps)", _limiter.LastValue().value());  
  frc::SmartDashboard::PutNumber("Climber/Load(A)", std::abs(_feedback.force.value()));
  frc::SmartDashboard::PutNumber("Climber/Position", _feedback.position.value());
}

// Helper function for configuring hardware from within the constructor of the subsystem.
bool Climber::ConfigureHardware() {
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
  configs.Slot0.kS = 0.0;

  // Slot 1 for the position control loop:
  configs.Slot1.kV = 0.12;
  configs.Slot1.kP = 0.3;
  configs.Slot1.kI = 0.03;
  configs.Slot1.kD = 0.01;
  configs.Slot1.kA = 0.0;
  configs.Slot1.kS = 0.03;

  
  // Set whether motor control direction is inverted or not:
  configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

  // Set the control configuration for the drive motor:
  auto status = _climberMotor.GetConfigurator().Apply(configs, 1_s); // 1 Second configuration timeout.

  if (!status.IsOK()) {
      std::cerr << "Climber: config failed to config!" << std::endl;
      return false;
  }

  _climberMotor.SetPosition(units::angle::turn_t(0));

  // Set our neutral mode to brake on:
  status = _climberMotor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

  if (!status.IsOK()) {
      std::cerr << "Climber: neutral mode failed to config :(!" << std::endl;
      return false;
  }

  // Log errors.
  return true;
}
