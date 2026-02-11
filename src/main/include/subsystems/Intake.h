// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/filter/SlewRateLimiter.h>

#include <variant>

class Intake : public frc2::SubsystemBase {
    
    public:

    static const ctre::phoenix6::CANBus canBus;

    struct Feedback {
        units::length::meter_t position;
        units::velocity::meters_per_second_t velocity;
    };

    using Command = std::variant<std::monostate, units::velocity::meters_per_second_t, units::length::meter_t>;

    Intake();

    const Feedback& GetFeedback() const { return _feedback; }

    void Periodic() override;
    
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> GetIntakeVelocity();

    units::angular_velocity::turns_per_second_t GetIntakeTargetVelocity();

    void SetIntakeVelocity(units::angular_velocity::turns_per_second_t Velocity);

    void SetCommand(Command cmd);

    private:

    static constexpr int _ActuatorLeadMotorID = 18; 
    static constexpr int _CollectorMotorID = 20; 
    static constexpr int _ActuatorFollowMotorID = 19;

    static constexpr auto ActuatorTurnsPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0); //TODO: Ratio
    static constexpr auto CollectorTurnPerMeter = units::angle::turn_t(32.0) / units::length::meter_t(1.0); //TODO: Ratio
    static constexpr auto ActuatorAmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0); //TODO: Ratio
    static constexpr auto CollectorAmpsPerNewton = units::current::ampere_t(10.0) / units::force::newton_t(1.0); //TODO: Ratio


    bool ConfigureHardware();
    bool _hardwareConfigured;

    ctre::phoenix6::hardware::TalonFX _ActuatorLeadMotor;
    ctre::phoenix6::hardware::TalonFX _ActuatorFollowMotor;
    ctre::phoenix6::hardware::TalonFX _CollectorMotor;

    ctre::phoenix6::StatusSignal<units::angle::turn_t> _PositionSig;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _ActuatorVelocitySig;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> _CollectorVelocitySig;
    ctre::phoenix6::StatusSignal<units::current::ampere_t> _CurrentSig;

    ctre::phoenix6::StatusSignal<units::volt_t> _voltageSignal = _ActuatorLeadMotor.GetMotorVoltage();

    ctre::phoenix6::controls::VelocityVoltage _ActuatorVelocityVoltage; //Slot 0
    ctre::phoenix6::controls::VelocityVoltage _CollectorVelocityVoltage; //Slot 0
    ctre::phoenix6::controls::PositionVoltage _PositionVoltage; //Slot 1

    units::angular_velocity::turns_per_second_t _TargetVelocity;

    frc::SlewRateLimiter<units::turns_per_second> limiter{0.5_tps / 1_s};

    Feedback _feedback;

    Command _command;

};