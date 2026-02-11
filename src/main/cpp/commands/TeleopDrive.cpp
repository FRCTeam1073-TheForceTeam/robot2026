// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopDrive.h"
#include <frc/DriverStation.h>
#include <iostream>

#include <choreo/Choreo.h>
#include <commands/Autos/TestAuto.h>


TeleopDrive::TeleopDrive(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<OI> oi, std::shared_ptr<Localizer> localizer) : 
    m_drivetrain{drivetrain}, 
    m_OI{oi},
    m_localizer{localizer} {
    allianceSign = 1,
    fieldCentric = true,
    lastParkingBreakButton = false,
    lastFieldCentricButton = true,
    parked = false,
    last_error = 0,
    last_snap_time = 0,
    angle_tolerance = 0.05_rad,
    torqueGate = 65_N,
    // TODO: chassisspeeds and speeds appear in the java drivetrain; determine if these are necessary for the c++ file
    // TODO: pointAtTarget boolean, localizer, lidar and aprilTagFinder appears in the java drivetrain, but it might be a better idea to put these in the localize file
    // Register that this command requires the subsystem.
    AddRequirements({m_drivetrain.get(), m_OI.get()});
}

void TeleopDrive::Initialize() {
    std::cerr << "TeleopDrive Init" << std::endl;
    Command::Initialize();
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        allianceSign = 1;
    }
    else {
        allianceSign = -1;
    }
}

void TeleopDrive::Execute() {
    leftY = m_OI->GetDriverLeftY();
    leftX = m_OI->GetDriverLeftX();
    rightX =  m_OI->GetDriverRightX();
    avgTorque = m_drivetrain->GetAverageLoad();
    currentTime = frc::Timer::GetMatchTime();


    frc::SmartDashboard::PutBoolean("TeleopDrive/Parking Break", parked);

    if(m_OI->GetDriverRightBumper() && lastFieldCentricButton == false) {
        fieldCentric = !fieldCentric;
    }
    lastFieldCentricButton = m_OI->GetDriverRightBumper();

    if(m_OI->GetDriverLeftBumper() && lastParkingBreakButton == false) {
        parked = !parked;
    }
    lastParkingBreakButton = m_OI->GetDriverLeftBumper();

    if(parked && !m_drivetrain->GetParkingBrake()) {
        m_drivetrain->SetParkingBrake(true);
    }
    if(!parked && m_drivetrain->GetParkingBrake()) {
        m_drivetrain->SetParkingBrake(false);
    }
    else {
        bool driverDPadUp = m_OI->GetDriverDPadUp();
        bool driverDPadDown = m_OI->GetDriverDPadDown();
        bool driverDPadLeft = m_OI->GetDriverDPadLeft();
        bool driverDPadRight = m_OI->GetDriverDPadRight();
        int driverDPadAngle = m_OI->GetDriverDPadAngle();

        mult1 = 1.0 + (m_OI->GetDriverLeftTrigger() * ((std::sqrt(36)) - 1));
        mult2 = 1.0 + (m_OI->GetDriverRightTrigger() * ((std::sqrt(36)) - 1));

        //set deadzones
        if(std::abs(leftY) < 0.15) {leftY = 0;}
        if(std::abs(leftX) < 0.15) {leftX = 0;}
        if(std::abs(rightX) < 0.15) {rightX = 0;}

        vx = std::clamp((allianceSign * leftY * maximumLinearVelocity / 25) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        vy = std::clamp((allianceSign * leftX * maximumLinearVelocity / 25) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        omega = std::clamp((rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);

        frc::SmartDashboard::PutNumber("TeleopDrive/vx", vx.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/vy", vy.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/omega", omega.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/AvgTorque", avgTorque.value());
        frc::SmartDashboard::PutBoolean("TeleopDrive/FieldCentric", fieldCentric);
        frc::SmartDashboard::PutNumber("TeleopDrive/leftX", leftX);
        frc::SmartDashboard::PutNumber("TeleopDrive/leftY", leftY);
        frc::SmartDashboard::PutNumber("TeleopDrive/rightX", rightX);
        frc::SmartDashboard::PutNumber("TeleopDrive/Driver DPad angle", driverDPadAngle);
        frc::SmartDashboard::PutBoolean("TeleopDrive/Driver DPad Up", driverDPadUp);
        frc::SmartDashboard::PutBoolean("TeleopDrive/Driver DPad Down", driverDPadDown);
        frc::SmartDashboard::PutBoolean("TeleopDrive/Driver DPad Left", driverDPadLeft);
        frc::SmartDashboard::PutBoolean("TeleopDrive/Driver DPad Right", driverDPadRight);

        // odometry centric drive
        if(fieldCentric) {
            m_drivetrain->SetChassisSpeeds(
                frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    vx,
                    vy,
                    omega,
                    //frc::Rotation2d{m_localizer->getPose().Rotation()} TODO:: figure out which one or both work
                    frc::Rotation2d{m_drivetrain->GetGyroHeadingRadians()}
                )
            );
        }
        else { // robot centric drive
            m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds{allianceSign * -vx, allianceSign * -vy, omega});
        }
        frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed Omega", m_drivetrain->GetChassisSpeeds().omega.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed X", m_drivetrain->GetChassisSpeeds().vx.value());
        frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed Y", m_drivetrain->GetChassisSpeeds().vy.value());

        auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Auto");
        frc::SmartDashboard::PutBoolean("TeleopDrive/A Button", m_OI->GetDriverAButton());
        if(m_OI->GetDriverAButton()) {
            TestAuto::Create(m_drivetrain, m_localizer, trajectory);
        }
    }

    if((((int)frc::Timer::GetMatchTime().value() - 30) % 25) == 0) {
        m_OI->DriverRumble();
    } else {
        m_OI->DriverStopRumble();
    }
}

void TeleopDrive::End(bool interrupted) {
    if(interrupted) {
        std::cerr << "TeleopDrive: Interrupted!" << std::endl;
    }
    Command::End(interrupted);
}

bool TeleopDrive::IsFinished() {
   return false;
}