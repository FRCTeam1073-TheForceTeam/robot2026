// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include <iostream>
#include <frc/kinematics/ChassisSpeeds.h>

using namespace ctre::phoenix6;
using namespace ctre::phoenix;

const CANBus Drivetrain::canBus(CANBus::RoboRIO());
bool debug = false;

Drivetrain::Drivetrain() :
    _imu(PigeonId, canBus),
    _swerveModules{
        SwerveModule({ 0,  7, 8, 6}, frc::Translation2d(0.254_m, 0.254_m), canBus), 
        SwerveModule({ 1,  10, 11, 9}, frc::Translation2d(0.254_m, -0.254_m), canBus), 
        SwerveModule({ 2,  13, 14, 12}, frc::Translation2d(-0.254_m, 0.254_m), canBus), 
        SwerveModule({ 3,  16, 17, 15}, frc::Translation2d(-0.254_m, -0.254_m), canBus)},
    _swerveModulePositions{_swerveModules[0].GetPosition(),
                _swerveModules[1].GetPosition(),
                _swerveModules[2].GetPosition(),
                _swerveModules[3].GetPosition()},
    _swerveModuleStates{_swerveModules[0].GetState(),
        _swerveModules[1].GetState(),
        _swerveModules[2].GetState(),
        _swerveModules[3].GetState()},
    _kinematics(_swerveModules[0].GetLocation(), 
                 _swerveModules[1].GetLocation(), 
                 _swerveModules[2].GetLocation(), 
                 _swerveModules[3].GetLocation()),
    _odometry(_kinematics, frc::Rotation2d(), 
                {_swerveModules[0].GetPosition(),
                _swerveModules[1].GetPosition(),
                _swerveModules[2].GetPosition(),
                _swerveModules[3].GetPosition()}, 
                frc::Pose2d()),
    _yawSig(_imu.GetYaw()),
    _pitchSig(_imu.GetPitch()),
    _rollSig(_imu.GetRoll()),
    _yawRateSig(_imu.GetAngularVelocityZDevice()),
    _hardwareConfigured(false),
    _parkingBrake(false)
{

    _hardwareConfigured &= ConfigureHardware();

    for (size_t ii(0); ii < _swerveModules.size(); ++ii) {
        _hardwareConfigured &= _swerveModules[ii].HardwareConfigured();
    }

    if (_hardwareConfigured) {
        std::cerr << "!! Drivetrain hardware configuration error !!" << std::endl;
    }    
}

void Drivetrain::Periodic()  {
    // Sample all the hardware:
    auto now = frc::Timer::GetFPGATimestamp();

    // Sample IMU signals and latency compensate the IMU data:
    BaseStatusSignal::RefreshAll(_yawSig, _yawRateSig, _pitchSig, _rollSig);

    // Latency compensated yaw:
    auto compensatedYaw = BaseStatusSignal::GetLatencyCompensatedValue(_yawSig, _yawRateSig);
    auto yaw_angle = frc::Rotation2d(compensatedYaw);

    for (size_t ii(0); ii < _swerveModules.size(); ++ii) {
        _swerveModules[ii].SampleFeedback(now);
        _swerveModulePositions[ii] = _swerveModules[ii].GetPosition();
        _swerveModuleStates[ii] = _swerveModules[ii].GetState(); // getState() returns SwerveModuleStates
        // _swerveModuleStates[ii]
        // std::cerr << "swervemodulestates index " << ii << " speed: " << _swerveModuleStates[ii].speed.value() << std::endl;
        // std::cerr << "swervemodulestates index " << ii << " speed: " << _swerveModuleStates[ii].angle.Degrees().value() << std::endl;
    }



    // Update odometry and chassis speeds once we've sampled the hardware:
    _speeds = _kinematics.ToChassisSpeeds(_swerveModuleStates); // this is frc::SwerveDriveKinematics.ToChassisSpeeds, not regular frc::Kinematics
    _odometry.Update(yaw_angle, _swerveModulePositions);

    // TODO: Compute extra pushing forces on the chassis based on torque feedback

    if (_parkingBrake) {
        // Hardcoded brake mode:
        // Leaves brake command on, does nothing here.
    } else {
        // Normal command pathway:
        auto moduleCommands = _kinematics.ToSwerveModuleStates(_targetSpeeds);
        frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&moduleCommands, SwerveControlConfig::MaxModuleSpeed);  // Keep commands feasible.

        // Send the commands to swerve module hardware:
        for (size_t ii(0); ii < _swerveModules.size(); ++ii) {
            moduleCommands[ii].Optimize(_swerveModulePositions[ii].angle);
            _swerveModules[ii].SetCommand(moduleCommands[ii]);
        }
    }

}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder) {
    // possibly remove this
    builder.AddDoubleProperty("Parking Break", [this] {return GetParkingBrake(); }, nullptr);
    builder.AddDoubleProperty("Odo X", [this] {return GetOdometry().X().value(); }, nullptr);
    builder.AddDoubleProperty("Odo Y", [this] {return GetOdometry().Y().value(); }, nullptr);
    builder.AddDoubleProperty("Odo Theta (Radians)", [this] {return GetOdometry().Rotation().Radians().value(); }, nullptr);
    builder.AddDoubleProperty("Odo Gyro Heading (Degrees)", [this] {return GetGyroHeadingDegrees().value(); }, nullptr);
    builder.AddDoubleProperty("Target Vx", _targetSpeeds.vx, nullptr);
    builder.AddDoubleProperty("Target Vy", _targetSpeeds.vy, nullptr);
    builder.AddDoubleProperty("Target Omega", _targetSpeeds.omega, nullptr);
    builder.AddDoubleProperty("Pitch", [this] {return GetPitch().value(); }, nullptr);
    builder.AddDoubleProperty("Roll", [this] {return GetRoll().value(); }, nullptr);
}

/// Reset the odometry to a specific pose on the field.
void Drivetrain::ResetOdometry(const frc::Pose2d pose) {
    _odometry.ResetPosition(_yawSig.GetValue(), 
                {_swerveModules[0].GetPosition(),
                _swerveModules[1].GetPosition(),
                _swerveModules[2].GetPosition(),
                _swerveModules[3].GetPosition()}, 
                pose);
}

void Drivetrain::SetParkingBrake(bool brakeOn) {

    if (brakeOn) {
        // Command modules to braked state:
        // Zero target:
        _targetSpeeds.vx = 0.0_mps;
        _targetSpeeds.vy = 0.0_mps;
        _targetSpeeds.omega = 0.0_rad/1.0_s;

        _swerveModules[0].SetCommand(frc::SwerveModuleState(0.0_mps, units::angle::degree_t(45.0)));
        _swerveModules[1].SetCommand(frc::SwerveModuleState(0.0_mps, units::angle::degree_t(-45.0)));
        _swerveModules[2].SetCommand(frc::SwerveModuleState(0.0_mps, units::angle::degree_t(-45.0)));
        _swerveModules[3].SetCommand(frc::SwerveModuleState(0.0_mps, units::angle::degree_t(45.0)));
    }

    _parkingBrake = brakeOn;
}

units::force::newton_t Drivetrain::GetAverageLoad() const {
    return 0.0_N;
}

units::angle::degree_t Drivetrain::GetGyroHeadingDegrees(){
    return _imu.GetYaw().Refresh().GetValue();
}

units::angle::radian_t Drivetrain::GetGyroHeadingRadians(){
    return _imu.GetYaw().Refresh().GetValue();
}

bool Drivetrain::ConfigureHardware() {
    configs::Pigeon2Configuration configs;

    // Apply configuration waiting for confirmation.
    auto status = _imu.GetConfigurator().Apply(configs, 1_s);

    if (!status.IsOK()) {
        std::cerr << "Drivetrain: IMU Hardware configuration error." << std::endl;
        return false;
    }

    // Set IMU readings for 100Hz.
    BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz,_yawRateSig, _yawSig);

    return true;
}

void Drivetrain::SetDebugMode(bool removeBug) {
    debug = removeBug;
}

void Drivetrain::ZeroHeading(){
    _imu.SetYaw(0_deg);
}

frc::Rotation2d Drivetrain::GetGyroHeading(){
    return _imu.GetRotation2d();
}