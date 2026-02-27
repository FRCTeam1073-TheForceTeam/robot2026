// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopDrive.h"
#include <frc/DriverStation.h>
#include <iostream>

#include <choreo/Choreo.h>
#include <commands/Autos/TestAuto.h>
#include <subsystems/OI.h>


TeleopDrive::TeleopDrive(std::shared_ptr<Drivetrain>& drivetrain, std::shared_ptr<OI>& oi, std::shared_ptr<Localizer>& localizer) : 
    m_drivetrain(drivetrain), 
    m_OI(oi),
    m_localizer(localizer)
    {
    allianceSign = 0;
    fieldCentric = true;
    lastParkingBreakButton = false;
    lastFieldCentricButton = true;
    parked = false;
    last_error = 0;
    last_snap_time = 0;
    angle_tolerance = 0.05_rad;
    torqueGate = 65_N;
    // TODO: chassisspeeds and speeds appear in the java drivetrain; determine if these are necessary for the c++ file
    // TODO: pointAtTarget boolean, localizer, lidar and aprilTagFinder appears in the java drivetrain, but it might be a better idea to put these in the localize file

    // Register that this command requires the subsystem.
    AddRequirements(m_drivetrain.get());
}


TeleopDrive::TeleopDrive(std::shared_ptr<Drivetrain>& drivetrain, std::shared_ptr<OI>& oi) : 
    m_drivetrain(drivetrain), 
    m_OI(oi) {
        
    allianceSign = 0;
    fieldCentric = true;
    lastParkingBreakButton = false;
    lastFieldCentricButton = true;
    parked = false;
    last_error = 0;
    last_snap_time = 0;
    angle_tolerance = 0.05_rad;
    torqueGate = 65_N;
    // TODO: chassisspeeds and speeds appear in the java drivetrain; determine if these are necessary for the c++ file
    // TODO: pointAtTarget boolean, localizer, lidar and aprilTagFinder appears in the java drivetrain, but it might be a better idea to put these in the localize file

    // Register that this command requires the subsystem.
    AddRequirements(m_drivetrain.get());
}

void TeleopDrive::Initialize() {
    std::cerr << "TeleopDrive Init" << std::endl;
    setAlliance();  // Set the alliance sign.
}


void TeleopDrive::Execute() {

    if (allianceSign == 0) setAlliance(); // Try to figure it out if not already set.

    double leftY = m_OI->GetDriverLeftY();
    double leftX = m_OI->GetDriverLeftX();
    double rightX =  m_OI->GetDriverRightX();
    avgTorque = m_drivetrain->GetAverageLoad();
    currentTime = frc::Timer::GetMatchTime();

    frc::SmartDashboard::PutBoolean("TeleopDrive/Parking Brake", parked);

    if (m_OI->GetDriverRightBumper() && lastFieldCentricButton == false) {
        fieldCentric = !fieldCentric;
    }
    lastFieldCentricButton = m_OI->GetDriverRightBumper();

    if (m_OI->GetDriverLeftBumper() && lastParkingBreakButton == false) {
        parked = !parked;
    }
    lastParkingBreakButton = m_OI->GetDriverLeftBumper();

    if (parked && !m_drivetrain->GetParkingBrake()) {
        m_drivetrain->SetParkingBrake(true);
    }

    if (!parked && m_drivetrain->GetParkingBrake()) {
        m_drivetrain->SetParkingBrake(false);
    }
    else {
        bool driverDPadUp = m_OI->GetDriverDPadUp();
        bool driverDPadDown = m_OI->GetDriverDPadDown();
        bool driverDPadLeft = m_OI->GetDriverDPadLeft();
        bool driverDPadRight = m_OI->GetDriverDPadRight();
        int driverDPadAngle = m_OI->GetDriverDPadAngle();

        double mult1 = 1.0 + (m_OI->GetDriverLeftTrigger() * ((std::sqrt(36)) - 1));
        double mult2 = 1.0 + (m_OI->GetDriverRightTrigger() * ((std::sqrt(36)) - 1));

        //set deadzones
        if (std::abs(leftY) < JOYSTICK_DEADZONE) {leftY = 0.0;}
        if (std::abs(leftX) < JOYSTICK_DEADZONE) {leftX = 0.0;}
        if (std::abs(rightX) < JOYSTICK_DEADZONE) {rightX = 0.0;}

        auto vx = std::clamp((allianceSign * leftY * maximumLinearVelocity / 25) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        auto vy = std::clamp((allianceSign * leftX * maximumLinearVelocity / 25) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        auto omega = std::clamp((rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);

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
        if (fieldCentric) {
            frc::Rotation2d rotation;
            if (m_localizer) {
                rotation = m_localizer->getPose().Rotation();
            } else {
                rotation = m_drivetrain->GetGyroHeading();
            }

            speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, rotation);
            m_drivetrain->SetChassisSpeeds(speeds);
        }
        else { // robot centric drive
            m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds{allianceSign * -vx, allianceSign * -vy, omega});
        }
        // Drivetrain should put out this information, not the command:
        // frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed Omega", m_drivetrain->GetChassisSpeeds().omega.value());
        // frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed X", m_drivetrain->GetChassisSpeeds().vx.value());
        // frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed Y", m_drivetrain->GetChassisSpeeds().vy.value());
    }

    // TODO: Put this logic in OI
    // if ((((int)frc::Timer::GetMatchTime().value() - 30) % 25) == 0) {
    //     m_OI->DriverRumble();
    // } else {
    //     m_OI->DriverStopRumble();
    // }
}

void TeleopDrive::End(bool interrupted) {
    if (interrupted) {
        std::cerr << "TeleopDrive: Interrupted!" << std::endl;
    }
}

bool TeleopDrive::IsFinished() {
   return false;
}

void TeleopDrive::setAlliance() {
    auto alliance = frc::DriverStation::GetAlliance();

    if (alliance.has_value()) {
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
            allianceSign = 1;
            std::cerr << "TeleopDrive:: RED Alliance" << std::endl;
        }
        else {
            allianceSign = -1;
            std::cerr << "TeleopDrive:: BLUE Alliance" << std::endl;
        }
    } else {
        std::cerr << "WARNING: TeleopDrive:: Alliance not set." << std::endl;
    }
}
