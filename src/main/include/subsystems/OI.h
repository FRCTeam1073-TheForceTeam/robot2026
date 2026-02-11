#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/force.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <variant>
#include <frc/filter/Debouncer.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/GenericHID.h>

class OI : public frc2::SubsystemBase {
    public:
        //probably not needed
        enum class Buttons{
            AButton = 0,
            BButton = 1,
            XButton = 2,
            YButton = 3,
            LeftJoystickY = 1,
            LeftJoystickX = 0,
            LeftJoystickPress = 8,
            RightJoystickY = 5,
            RightJoystickX = 4,
            RightJoystickPress = 9,
            DpadUp = 0,
            DpadLeft = 270,
            DpadRight = 90,
            DpadDown = 180,
            LeftBumper = 4,
            RightBumper = 5,
            LeftTrigger = 2,
            RightTrigger = 3,
            ViewButton = 6,
            MenuButton = 7
        };

        //TODO: do we want a stick double value to radians
        
        // feedback
        struct Feedback {
            //TODO: figure out what to put in here
        };

        // Constructor
        OI();

        void Periodic() override;
        
        // access the feedback from the controllers
        const Feedback& GetFeedback() const { return _feedback; }

        double GetDriverLeftX();
        double GetDriverLeftY();
        double GetDriverRightX();
        double GetDriverRightY();
        double GetOperatorLeftX();
        double GetOperatorLeftY();
        double GetOperatorRightX();
        double GetOperatorRightY();

        double GetDriverLeftTrigger();
        double GetDriverRightTrigger();
        double GetOperatorLeftTrigger();
        double GetOperatorRightTrigger();

        bool GetDriverAButton();
        bool GetDriverBButton();
        bool GetDriverXButton();
        bool GetDriverYButton();
        bool GetDriverMenuButton();
        bool GetDriverViewButton();
        bool GetDriverLeftBumper();
        bool GetDriverRightBumper();
        int GetDriverDPadAngle();
        bool GetDriverDPadUp();
        bool GetDriverDPadRight();
        bool GetDriverDPadLeft();
        bool GetDriverDPadDown();
        bool DriverRumble();
        bool DriverStopRumble();

        bool GetOperatorAButton();
        bool GetOperatorBButton();
        bool GetOperatorXButton();
        bool GetOperatorYButton();
        bool GetOperatorMenuButton();
        bool GetOperatorViewButton();
        bool GetOperatorLeftBumper();
        bool GetOperatorRightBumper();
        int GetOperatorDPadAngle();
        bool GetOperatorDPadUp();
        bool GetOperatorDPadRight();
        bool GetOperatorDPadLeft();
        bool GetOperatorDPadDown();
        bool OperatorRumble();
        bool OperatorStopRumble();

        void ZeroDriverController();
        void ZeroOperatorController();

    private:
        Feedback _feedback;

        // create the controller objects
        frc::XboxController _driverController;
        frc::XboxController _operatorController;    

        // TODO: make debouncers for indevidual buttons
        // general debouncer for buttons
        frc::Debouncer m_debouncer{50_ms, frc::Debouncer::DebounceType::kBoth};


        double LEFT_X_ZERO;
        double LEFT_Y_ZERO;
        double RIGHT_X_ZERO;
        double RIGHT_Y_ZERO;
};