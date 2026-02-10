#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/OI.h"
#include "subsystems/Localizer.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <memory>

class TeleopDrive
    : public frc2::CommandHelper<frc2::Command, TeleopDrive> {
public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
    explicit TeleopDrive(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<OI> oi, std::shared_ptr<Localizer> localizer);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;

private:
    std::shared_ptr<Drivetrain> m_drivetrain;
    std::shared_ptr<OI> m_OI;
    std::shared_ptr<Localizer> m_localizer;

    frc::ChassisSpeeds speeds;

    units::angle::radian_t angle_tolerance;
    frc::PIDController snapPIDProfile{0.05, 0.0, 0.0, 1_s};

    bool fieldCentric;
    bool parked;

    double last_error;
    double last_snap_time;

    bool lastParkingBreakButton;
    bool lastFieldCentricButton;

    static constexpr units::velocity::meters_per_second_t maximumLinearVelocity = 3.5_mps;
    static constexpr units::angular_velocity::radians_per_second_t maximumRotationVelocity = 4.0_rad_per_s;

    double mult1;
    double mult2;

    double leftX;
    double leftY;
    double rightX;

    units::velocity::meters_per_second_t vx;
    units::velocity::meters_per_second_t vy;
    units::angular_velocity::radians_per_second_t omega;

    int allianceSign;

    units::force::newton_t frontLeftTorque;
    units::force::newton_t frontRightTorque;
    units::force::newton_t backLeftTorque;
    units::force::newton_t backRightTorque;
    units::force::newton_t avgTorque;
    units::force::newton_t torqueGate;

    //TODO: aprilTagFinde, Localizer, and Lidar stuff

};
