 #include "subsystems/Climber.h"

 #include <iostream>
 #include <ctre/phoenix6/controls/NeutralOut.hpp>

 using namespace ctre::phoenix6;

 Climber::Climber() :
    _hardwareConfigured(true),
    _climberOn(false),
    _Motor(MotorId, CANBus("rio")),
    _VelocitySig(_Motor.GetVelocity()),
    _CurrentSig(_Motor.GetTorqueCurrent()),
    _PositionSig(_Motor.GetPosition()),
    _VelocityVoltage(units::angular_velocity::turns_per_second_t(0.0)),
    _PositionVoltage(units::angle::turn_t(0.0)) {
    _VelocityVoltage.WithSlot(0);
 
    _hardwareConfigured = ConfigureHardware();

    if (!_hardwareConfigured) {
        std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
    }
};

void Climber::SetCommand(Command cmd) {
  // Sometimes you need to do something immediate to the hardware.
  // We can just set our target internal value.
  _command = cmd;
}

void Climber::SetVelocity(units::angular_velocity::turns_per_second_t Velocity) {
    _TargetVelocity = Velocity;
}
void Climber::SetPosition(units::length::meter_t pos){
  TargetPosition = pos;
}
ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> Climber::GetVelocity() {
    return _VelocitySig;
}
units::angular_velocity::turns_per_second_t Climber::GetTargetVelocity() {
    return _TargetVelocity;
}
units::length::meter_t Climber::GetPosition() {
  return Position;
}
units::length::meter_t Climber::GetTargetPosition() {
  return TargetPosition;
}
void Climber::SetVoltage(units::volt_t Voltage) {
    _Motor.SetVoltage(Voltage);
}
units::volt_t Climber::GetVoltage() {
    return _voltageSignal.GetValue();
}
void Climber::StopMotor() {
    _Motor.StopMotor();
}
  


bool Climber::IsHooked() {
  if (m_ClimberOnInput.Get()) {
      _climberOn = true; // these may be swapped depending on the digitalinput default is //
    } else {
      _climberOn = false;
    }
  return _climberOn;
}


void Climber::Periodic() {
    BaseStatusSignal::RefreshAll(_VelocitySig, _CurrentSig);

    _feedback.force = _CurrentSig.GetValue() / AmpsPerNewton;
    _feedback.velocity = _VelocitySig.GetValue() / TurnsPerMeter;

    RotationPosition = _PositionSig.GetValue();
    Position = RotationPosition / TurnsPerMeter;
    _feedback.position = Position;


    if (m_ClimberOnInput.Get()) {
      _climberOn = true; // these may be swapped depending on the digitalinput default is
    } else {
      _climberOn = false;
    }
    
    if (std::holds_alternative<units::velocity::meters_per_second_t>(_command)) {
      // Send velocity based command:

      // Convert to hardware units:
      // Multiply by conversion to produce commands.
      auto angular_vel = std::get<units::velocity::meters_per_second_t>(_command) * TurnsPerMeter;
      // Send to hardware:
      _Motor.SetControl(_VelocityVoltage.WithVelocity(limiter.Calculate(angular_vel)));
    } else {
      // No command, so send a "null" neutral output command if there is no position or velocity provided as a command:
    _Motor.SetControl(controls::NeutralOut());
    }
}

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

    // Set whether motor control direction is inverted or not:
    configs.MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    // Set the control configuration for the drive motor:
    auto status = _Motor.GetConfigurator().Apply(configs, 1_s ); // 1 Second configuration timeout.

    // Set our neutral mode to brake on:
    status = _Motor.SetNeutralMode(signals::NeutralModeValue::Brake, 1_s);

    if (!status.IsOK()) {
        std::cerr << "ExampleSubsystem: Hardware Failed To Configure!" << std::endl;
    }

    // Log errors.
    return false;
}
    