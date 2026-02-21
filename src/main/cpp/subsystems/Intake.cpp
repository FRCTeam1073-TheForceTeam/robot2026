// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include <iostream>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6;
using namespace units;


Intake::Intake(): 
    _hardwareConfigured(true),
    _ActuatorLeadMotor(_ActuatorLeadMotorID, CANBus("rio")), 
    _ActuatorFollowMotor(_ActuatorFollowMotorID, CANBus("rio")), 
    _PositionSig(_ActuatorLeadMotor.GetPosition()),
    _ActuatorVelocitySig(_ActuatorLeadMotor.GetVelocity()),
    _CurrentSig(_ActuatorLeadMotor.GetTorqueCurrent()),
    _ActuatorVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)), //TODO: Get Velocity
    _PositionVoltage(units::angle::turn_t(0.0)) //TODO: Get Velocity
{
    _ActuatorVelocityVoltage.WithSlot(0);
    _PositionVoltage.WithSlot(1);

    _hardwareConfigured = ConfigureHardware();
    if (!_hardwareConfigured) {
        std::cerr << "Intake: Hardware Failed To Configure!" << std::endl;
    }

    _command = std::monostate();

} 

void Intake::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Intake::SetIntakeVelocity(units::angular_velocity::turns_per_second_t Velocity) {
  _TargetVelocity = Velocity;
}

ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> Intake::GetIntakeVelocity() {
  return _ActuatorVelocitySig;
}

units::angular_velocity::turns_per_second_t Intake::GetIntakeTargetVelocity() {
  return _TargetVelocity;
}

void Intake::Periodic() {
    
    BaseStatusSignal::RefreshAll(_PositionSig, _ActuatorVelocitySig, _CurrentSig);

    _feedback.velocity = _ActuatorVelocitySig.GetValue() /ActuatorTurnsPerMeter;

    _ActuatorFollowMotor.SetControl(controls::StrictFollower{_ActuatorLeadMotor.GetDeviceID()});

  if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
  
    // TODO:

  } else {

    // TODO:
  }

}

bool Intake::ConfigureHardware() {
    configs::TalonFXConfiguration configs{};

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10.0_A; //TODO: Get Peak
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10.0_A; //TODO: Get Peak

    configs.Voltage.PeakForwardVoltage = 8_V; //TODO: Get Peak
    configs.Voltage.PeakReverseVoltage = -8_V; //TODO: Geat Peak

    // Slot 0, TODO: Get Numbers
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.35;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.03;
    configs.Slot0.kA = 0.0;

    // Slot 1, TODO: Get Numbers
    configs.Slot1.kV = 0.12;
    configs.Slot1.kP = 0.1;
    configs.Slot1.kI = 0.01;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kA = 0.0;

    auto status = _ActuatorLeadMotor.GetConfigurator().Apply(configs, 1_s);

    configs::TalonFXConfiguration followerConfigs{};
    followerConfigs.MotorOutput.WithInverted(signals::InvertedValue::CounterClockwise_Positive);
    
    if (!status.IsOK()) {
        std::cerr << "Intake is not working" << std::endl;
    }

    return true;

}