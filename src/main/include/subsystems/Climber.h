#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>
#include <units/voltage.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <variant>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/DigitalInput.h>

class Climber : public frc2::SubsystemBase {
 public:

  // CANBusID for the motor.
  static constexpr int MotorId = 29;

  // Mechanism conversion constants for the subsystem:
  static constexpr auto TurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0);//TODO: get values for this from EM
  static constexpr auto AmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0);//TODO: get values for this from EM

  
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
      units::velocity::meters_per_second_t velocity;
      units::force::newton_t force;
      units::length::meter_t position;
  };

  using Command = std::variant<std::monostate, units::velocity::meters_per_second_t, units::length::meter_t>;

  Climber();

  void Periodic() override;


  /// Access the latest feedback from the system. 
  const Feedback& GetFeedback() const { return _feedback; }

  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> GetVelocity();

  units::angular_velocity::turns_per_second_t GetTargetVelocity();

  void SetVelocity(units::angular_velocity::turns_per_second_t Velocity);

  void SetPosition(units::length::meter_t pos);

  units::length::meter_t GetPosition();

  units::length::meter_t GetTargetPosition();
  bool IsHooked();

  void SetVoltage(units::volt_t Voltage);

  units::volt_t GetVoltage();

  void StopMotor();


  /// Set the command for the system.
  void SetCommand(Command cmd);

 private:
 
  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  bool _climberUp;

  bool _climberOn;

  // Example TalonFX motor interface.
  ctre::phoenix6::hardware::TalonFX _Motor;

  // CTRE hardware feedback signals:
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _VelocitySig;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> _CurrentSig;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> _PositionSig;
  ctre::phoenix6::StatusSignal<units::volt_t> _voltageSignal = _Motor.GetMotorVoltage();


  // Example velocity and position controls:
  ctre::phoenix6::controls::VelocityVoltage _VelocityVoltage;  // Uses Slot0 gains.

   ctre::phoenix6::controls::PositionVoltage _PositionVoltage;
  
  // Cached feedback:
  Feedback _feedback;

  // Cached command: Variant of possible different kinds of commands.
  Command  _command;

  units::angular_velocity::turns_per_second_t _TargetVelocity;

  units::length::meter_t TargetPosition;
  units::length::meter_t Position;

  units::angle::turn_t RotationPosition;
  units::angle::turn_t TargetRotationPosition;

  frc::SlewRateLimiter<units::turns_per_second> limiter{0.5_tps / 1_s};

  frc::DigitalInput m_ClimberOnInput{0};

};