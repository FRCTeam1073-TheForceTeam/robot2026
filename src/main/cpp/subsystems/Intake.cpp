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
    _CollectorMotor(_CollectorMotorID, CANBus("rio)")), 
    _ActuatorFollowMotor(_ActuatorFollowMotorID, CANBus("rio")), 
    _PositionSig(_ActuatorLeadMotor.GetPosition()),
    _ActuatorVelocitySig(_ActuatorLeadMotor.GetVelocity()),
    _CollectorVelocitySig(_CollectorMotor.GetVelocity()),
    _CurrentSig(_ActuatorLeadMotor.GetTorqueCurrent()),
    _ActuatorVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)), //TODO: Get Velocity
    _CollectorVelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)), //TODO: Get Velocity
    _PositionVoltage(units::angle::turn_t(0.0)) //TODO: Get Velocity
{
    _ActuatorVelocityVoltage.WithSlot(0);
    _CollectorVelocityVoltage.WithSlot(0);
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
    
    BaseStatusSignal::RefreshAll(_PositionSig, _ActuatorVelocitySig, _CollectorVelocitySig, _CurrentSig);

    // _feedback.velocity = _ActuatorVelocitySig.GetValue() /ActuatorTurnsPerMeter;
    // _feedback.velocity = _CollectorVelocitySig.GetValue() / CollectorTurnPerMeter;

    _ActuatorFollowMotor.SetControl(controls::StrictFollower{_ActuatorLeadMotor.GetDeviceID()});

    if (_voltageSignal.GetValue() > volt_t(5)) { //TODO: Get Value

      _ActuatorLeadMotor.SetVoltage(volt_t(0));
      _ActuatorLeadMotor.StopMotor();

    } 

  if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
  
    auto angular_vel = std::get<units::velocity::meters_per_second_t>(_command) * CollectorTurnPerMeter;
    _CollectorMotor.SetControl(_CollectorVelocityVoltage.WithVelocity(angular_vel));

  } else {

    _CollectorMotor.SetControl(controls::NeutralOut());

  }

  _CollectorMotor.Set(limiter.Calculate(_TargetVelocity).value());

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
    // auto status = _CollectorMotor.GetConfigurator().Apply(configs, 1_s);

    configs::TalonFXConfiguration followerConfigs{};
    followerConfigs.MotorOutput.WithInverted(signals::InvertedValue::CounterClockwise_Positive);

    _ActuatorLeadMotor.SetVoltage(volt_t(0));
    _CollectorMotor.SetVoltage(volt_t(0));
    
    if (!status.IsOK()) {
        std::cerr << "Intake is not working" << std::endl;
    }

    return true;

}