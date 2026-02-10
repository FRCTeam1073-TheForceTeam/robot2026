// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/OI.h"
#include <iostream>

OI::OI() :
_driverController(0),
_operatorController(1)
{}

// This method will be called once per scheduler run
void OI::Periodic() {}

double OI::GetDriverLeftX() {
    return _driverController.GetLeftX();
}

double OI::GetDriverLeftY() {
    return _driverController.GetLeftY();
}

double OI::GetDriverRightX() {
    return -1 * _driverController.GetRightX();
}

double OI::GetDriverRightY() {
    return _driverController.GetRightY();
}

double OI::GetOperatorLeftX() {
    return -1 * _operatorController.GetLeftX();
}

double OI::GetOperatorLeftY() {
    return _operatorController.GetLeftY();
}

double OI::GetOperatorRightX() {
    return _operatorController.GetRightX();
}

double OI::GetOperatorRightY() {
    return _operatorController.GetRightY();
}

double OI::GetDriverLeftTrigger() {
    return _driverController.GetLeftTriggerAxis();
}

double OI::GetDriverRightTrigger() {
    return _driverController.GetRightTriggerAxis();
}

double OI::GetOperatorLeftTrigger() {
    return _operatorController.GetRightTriggerAxis();
}

double OI::GetOperatorRightTrigger() {
    return _operatorController.GetRightTriggerAxis();
}

bool OI::GetDriverAButton() {
    return _driverController.GetAButton();
}

bool OI::GetDriverBButton() {
    return _driverController.GetBButton();
}

bool OI::GetDriverXButton() {
    return _driverController.GetXButton();
}

bool OI::GetDriverYButton() {
    return _driverController.GetYButton();
}

bool OI::GetDriverMenuButton() {
    return _driverController.GetStartButton();
}

bool OI::GetDriverViewButton() {
    return _driverController.GetBackButton();
}

bool OI::GetDriverLeftBumper() {
    return _driverController.GetLeftBumperButton();
}

bool OI::GetDriverRightBumper() {
    return _driverController.GetRightBumperButton();
}

int OI::GetDriverDPadAngle() {
    return _driverController.GetPOV();
}

bool OI::GetDriverDPadUp() {
    //return _driverController.GetPOV() == 0;
    return false;
}

bool OI::GetDriverDPadRight() {
    //return _driverController.GetPOV() == 90;
    return false;
}

bool OI::GetDriverDPadLeft() {
    //return _driverController.GetPOV() == 270;
    return false;
}

bool OI::GetDriverDPadDown() {
    //return _driverController.GetPOV(180) == 180;
    return false;
}

bool OI::GetOperatorAButton() {
    return _operatorController.GetAButton();
}

bool OI::GetOperatorBButton() {
    return _operatorController.GetBButton();
}

bool OI::GetOperatorXButton() {
    return _operatorController.GetXButton();
}

bool OI::GetOperatorYButton() {
    return _operatorController.GetYButton();
}

bool OI::GetOperatorMenuButton() {
    return _operatorController.GetStartButton();
}

bool OI::GetOperatorViewButton() {
    return _operatorController.GetBackButton();
}

bool OI::GetOperatorLeftBumper() {
    return _operatorController.GetLeftBumperButton();
}

bool OI::GetOperatorRightBumper() {
    return _operatorController.GetRightBumperButton();
}

bool OI::GetOperatorDPadUp() {
    //return _driverController.GetPOV() == 0;
    return false;
}

bool OI::GetOperatorDPadRight() {
    ///return _driverController.GetPOV() == 90;
    return false;
}

bool OI::GetOperatorDPadLeft() {
    /////return _driverController.GetPOV() == 270;
    return false;
}

bool OI::GetOperatorDPadDown() {
    ////////return _driverController.GetPOV() == 180;
    return false;
}

void OI::ZeroDriverController() {
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = GetDriverLeftX();
    LEFT_Y_ZERO = GetDriverLeftY();
    RIGHT_X_ZERO = GetDriverRightX();
    RIGHT_Y_ZERO = GetDriverRightY();
}

void OI::ZeroOperatorController() {
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = GetOperatorLeftX();
    LEFT_Y_ZERO = GetOperatorLeftY();
    RIGHT_X_ZERO = GetOperatorRightX();
    RIGHT_Y_ZERO = GetOperatorRightY();
}
