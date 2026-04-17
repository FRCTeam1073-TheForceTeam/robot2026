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
    m_localizer(localizer),
    thetaController{3.0, 0.2, 0.3}
    {
    allianceSign = 0;
    fieldCentric = true;
    // lastParkingBreakButton = false;
    lastFieldCentricButton = true;
    // parked = false;
    angle_tolerance = 0.05_rad;
    maximumRotationVelocity = 4.75_rad_per_s;
    torqueGate = 65_N;
    slowMode = false;
    lastYPressed = false;
    fastRotation = false;
    heading = 0_rad;
    // TODO: chassisspeeds and speeds appear in the java drivetrain; determine if these are necessary for the c++ file
    // TODO: pointAtTarget boolean, localizer, lidar and aprilTagFinder appears in the java drivetrain, but it might be a better idea to put these in the localize file

    // Register that this command requires the subsystem.
    thetaController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
    AddRequirements(m_drivetrain.get());
}


TeleopDrive::TeleopDrive(std::shared_ptr<Drivetrain>& drivetrain, std::shared_ptr<OI>& oi) : 
    m_drivetrain(drivetrain), 
    m_OI(oi),
    thetaController{3.0, 0.2, 0.3} {
    allianceSign = 0;
    fieldCentric = true;
    // lastParkingBreakButton = false;
    lastFieldCentricButton = true;
    // parked = false;
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
    heading = m_localizer->getPose().Rotation().Radians();
    thetaController.Reset();
}


void TeleopDrive::Execute() {

    if (allianceSign == 0) setAlliance(); // Try to figure it out if not already set.

    double leftY = m_OI->GetDriverLeftY();
    double leftX = m_OI->GetDriverLeftX();
    double rightX =  m_OI->GetDriverRightX();
    avgTorque = m_drivetrain->GetAverageLoad();
    currentTime = frc::Timer::GetMatchTime();

    // frc::SmartDashboard::PutBoolean("TeleopDrive/Parking Brake", parked);

    if (m_OI->GetDriverLeftBumper() && lastFieldCentricButton == false) {
        fieldCentric = !fieldCentric;
    }
    lastFieldCentricButton = m_OI->GetDriverLeftBumper();

    // if (m_OI->GetDriverLeftBumper() && lastParkingBreakButton == false) {
    //     parked = !parked;
    // }
    // lastParkingBreakButton = m_OI->GetDriverLeftBumper();

    // if (parked && !m_drivetrain->GetParkingBrake()) {
    //     m_drivetrain->SetParkingBrake(true);
    // }

    // if (!parked && m_drivetrain->GetParkingBrake()) {
    //     m_drivetrain->SetParkingBrake(false);
    // }
    bool driverDPadUp = m_OI->GetDriverDPadUp();
    bool driverDPadDown = m_OI->GetDriverDPadDown();
    bool driverDPadLeft = m_OI->GetDriverDPadLeft();
    bool driverDPadRight = m_OI->GetDriverDPadRight();
    int driverDPadAngle = m_OI->GetDriverDPadAngle();

    //set deadzones
    if (std::abs(leftY) < JOYSTICK_DEADZONE) {leftY = 0.0;}
    if (std::abs(leftX) < JOYSTICK_DEADZONE) {leftX = 0.0;}
    if (std::abs(rightX) < JOYSTICK_DEADZONE) {rightX = 0.0;}

    auto vx = std::clamp(allianceSign * ((leftY + 0.01) / std::abs(leftY + 0.01)) * (maximumLinearVelocity / (maximumLinearVelocity.value() - 1)) * (std::pow(maximumLinearVelocity.value(), std::abs(leftY)) - 1), -maximumLinearVelocity, maximumLinearVelocity);
    auto vy = std::clamp(allianceSign * ((leftX + 0.01) / std::abs(leftX + 0.01)) * (maximumLinearVelocity / (maximumLinearVelocity.value() - 1)) * (std::pow(maximumLinearVelocity.value(), std::abs(leftX)) - 1), -maximumLinearVelocity, maximumLinearVelocity);
    auto omega = std::clamp(((rightX + 0.01) / std::abs(rightX + 0.01)) * (maximumRotationVelocity / (maximumRotationVelocity.value() - 1)) * (std::pow(maximumRotationVelocity.value(), std::abs(rightX)) - 1), -maximumRotationVelocity, maximumRotationVelocity);
    
    if (slowMode) {
        vx *= 0.4;
        vy *= 0.4;
        omega *= 0.4;
    }

    auto delta = omega * 0.02_s;

    heading += frc::AngleModulus(delta);
    auto heading_omega = std::clamp(thetaController.Calculate(m_localizer->getPose().Rotation().Radians().value(), heading.value()), -maximumRotationVelocity.value(), maximumRotationVelocity.value()) * 1_rad_per_s;

    if (!lastYPressed && m_OI->GetDriverYButton()) {
        slowMode = !slowMode;
    }
    lastYPressed = m_OI->GetDriverYButton();

    if (m_OI->GetDriverXButton()) {
        maximumRotationVelocity = 5.5_rad_per_s;
    } else {
        maximumRotationVelocity = 4.75_rad_per_s;
    }

    if (maximumRotationVelocity == 5.5_rad_per_s) {
        fastRotation = true;
    } else {
        fastRotation = false;
    }

    frc::SmartDashboard::PutBoolean("TeleopDrive/Slow Mode", slowMode);
    frc::SmartDashboard::PutNumber("TeleopDrive/vx", vx.value());
    frc::SmartDashboard::PutNumber("TeleopDrive/vy", vy.value());
    frc::SmartDashboard::PutNumber("TeleopDrive/omega", omega.value());
    frc::SmartDashboard::PutNumber("TeleopDrive/Heading Omega", heading_omega.value());
    frc::SmartDashboard::PutNumber("TeleopDrive/Heading", frc::AngleModulus(heading).value());
    frc::SmartDashboard::PutNumber("Heading Delta", delta.value());
    frc::SmartDashboard::PutNumber("TeleopDrive/RightX", rightX);
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
    frc::SmartDashboard::PutNumber("TeleopDrive/Maximum Rotation Velocity", maximumRotationVelocity.value());
    frc::SmartDashboard::PutBoolean("TeleopDrive/Fast Rotation", fastRotation);

    // odometry centric drive
    if (fieldCentric) {
        frc::Rotation2d rotation;
        if (m_localizer) {
            rotation = m_localizer->getPose().Rotation();
        } else {
            rotation = m_drivetrain->GetGyroHeading();
        }

        speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, heading_omega, rotation);
        m_drivetrain->SetChassisSpeeds(speeds);
    }
    else { // robot centric drive
        m_drivetrain->SetChassisSpeeds(frc::ChassisSpeeds{allianceSign * -vx, allianceSign * -vy, omega});
    }
        // Drivetrain should put out this information, not the command:
        // frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed Omega", m_drivetrain->GetChassisSpeeds().omega.value());
        // frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed X", m_drivetrain->GetChassisSpeeds().vx.value());
        // frc::SmartDashboard::PutNumber("TeleopDrive/Chassis Speed Y", m_drivetrain->GetChassisSpeeds().vy.value());

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
