// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "subsystems/Flywheel.h"
#include <ctre/phoenix6/controls/NeutralOut.hpp>

using namespace ctre::phoenix6;

Flywheel::Flywheel(): 

    _leadFlywheelMotor(LeadMotorId, CANBus::RoboRIO()),
    _followFlywheelMotor(FollowMotorId, CANBus::RoboRIO()),
    _flywheelVelocitySig(_leadFlywheelMotor.GetVelocity()),
    _flywheelCurrentSig(_leadFlywheelMotor.GetTorqueCurrent()),
    _limiter(20_mps / 1_s),
    _flywheelVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
    _hardwareConfigured(false) {
        
    _flywheelVelocityVoltage.WithSlot(0);

    _hardwareConfigured = ConfigureHardware();
    if(!_hardwareConfigured) {
        std::cerr << "hardware failed to conifgure in shooter" << std::endl;
    }

    frc::SmartDashboard::PutNumber("Flywheel/hardware_configured", _hardwareConfigured);
}


units::velocity::meters_per_second_t Flywheel::GetTargetVelocity() {
    if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
        return std::get<units::velocity::meters_per_second_t>(_command);
    } else {
        return 0.0_mps;
    }
}

void Flywheel::SetCommand(Command cmd){
    _command = cmd;
}

// This method will be called once per scheduler run
void Flywheel::Periodic() {
    BaseStatusSignal::RefreshAll(_flywheelVelocitySig, _flywheelCurrentSig);

    _feedback.velocity = _flywheelVelocitySig.GetValue() / TurnsPerMeter; // Convert from hardare units to subsystem units.
    _feedback.force = _flywheelCurrentSig.GetValue() / AmpsPerNewton; // Convert from hardware units to subsystem units.

    if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {

        // Compute a rate-limited velocity:
        auto limited_velocity = _limiter.Calculate(std::get<units::velocity::meters_per_second_t>(_command)); 
        auto motor_velocity = limited_velocity * TurnsPerMeter;

        // Send commands to motors:
        _leadFlywheelMotor.SetControl(_flywheelVelocityVoltage.WithVelocity(motor_velocity));
        _followFlywheelMotor.SetControl(controls::StrictFollower{_leadFlywheelMotor.GetDeviceID()});

    } else {
        // Send commands to motors:
        _leadFlywheelMotor.SetControl(controls::NeutralOut());
        _followFlywheelMotor.SetControl(controls::NeutralOut());
        // Reset the limiter:
        _limiter.Reset(0.0_mps);
    }


    frc::SmartDashboard::PutNumber("Flywheel/AngularVelocity (RPM)", (60.0_s*_flywheelVelocitySig.GetValue()).value());
    frc::SmartDashboard::PutNumber("Flywheel/TargetVelocity (mps)", _limiter.LastValue().value());
    frc::SmartDashboard::PutNumber("Flywheel/Velocity (mps)", _feedback.velocity.value());

}

bool Flywheel::ConfigureHardware() {
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; // Set current limits to keep from breaking things.
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; 

    configs.Voltage.PeakForwardVoltage = 8_V; // These are pretty typical values, adjust as needed.
    configs.Voltage.PeakReverseVoltage = -8_V;

    // Slot 0 for the velocity control loop:
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.2;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.01;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.2;

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _leadFlywheelMotor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "Flywheel lead motor configfuration failed." << std::endl;
        return false;
    }
    _leadFlywheelMotor.SetNeutralMode(signals::NeutralModeValue::Coast, 1_s);

    configs::TalonFXConfiguration followerConfigs{};
    followerConfigs.MotorOutput.WithInverted(signals::InvertedValue::CounterClockwise_Positive); //change this if directions are the same.
    status = _followFlywheelMotor.GetConfigurator().Apply(followerConfigs, 1_s ); // 1 Second configuration timeout.

    if (!status.IsOK()) {
        std::cerr << "Flywheel follower motor configfuration failed." << std::endl;
        return false;
    }

    return true;
}
