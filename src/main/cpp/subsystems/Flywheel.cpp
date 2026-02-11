// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "subsystems/Flywheel.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

Flywheel::Flywheel(): 
    _hardwareConfigured(true), 
    _leadFlywheelMotor(LeadMotorId, "rio"),
    _followFlywheelMotor(FollowMotorId, "rio"),
    _FlywheelVelocitySig(_leadFlywheelMotor.GetVelocity()),
    _FlywheelCurrentSig(_leadFlywheelMotor.GetTorqueCurrent()),
    _FlywheelVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)) {
        
    _FlywheelVelocityVoltage.WithSlot(0);

    _hardwareConfigured = ConfigureHardware();
        if(!_hardwareConfigured) {
            std::cerr << "hardware failed to conifgure in shooter" << std::endl;
        }
    }

void Flywheel::SetVelocity(units::angular_velocity::turns_per_second_t Velocity) {
    _TargetVelocity = Velocity;
}
ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> Flywheel::GetVelocity() {
    return _FlywheelVelocitySig;
}
units::angular_velocity::turns_per_second_t Flywheel::GetTargetVelocity() {
    return _TargetVelocity;
}

void Flywheel::SetCommand(Command cmd){
    _command = cmd;
}


// This method will be called once per scheduler run
void Flywheel::Periodic() {
    BaseStatusSignal::RefreshAll(_FlywheelVelocitySig, _FlywheelCurrentSig);

    _feedback.velocity = _FlywheelVelocitySig.GetValue() / TurnsPerMeter; // Convert from hardare units to subsystem units.
    _feedback.force = _FlywheelCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.

    //_followFlywheelMotor.Set(_FlywheelVelocitySig.GetValue().value());

  if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
      // Send velocity based command:

      // Convert to hardware units:
      // Multiply by conversion to produce commands.
      auto angular_vel = std::get<units::velocity::meters_per_second_t>(_command) * TurnsPerMeter;
      // Send to hardware:
      _leadFlywheelMotor.SetControl(_FlywheelVelocityVoltage.WithVelocity(angular_vel));
  } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _leadFlywheelMotor.SetControl(controls::NeutralOut());
  }

     _followFlywheelMotor.SetControl(controls::StrictFollower{_leadFlywheelMotor.GetDeviceID()});

}

bool Flywheel::ConfigureHardware() {
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
    auto status = _leadFlywheelMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.
   
    configs::TalonFXConfiguration FollowerConfigs{};
    FollowerConfigs.MotorOutput.WithInverted(signals::InvertedValue::CounterClockwise_Positive); //change this if directions are the same.

    if (!status.IsOK()) {
        std::cerr << "Flywheel is not working" << std::endl;
    }

    return true;

}
