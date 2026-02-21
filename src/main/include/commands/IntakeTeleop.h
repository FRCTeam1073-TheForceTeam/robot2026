// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Intake.h"
#include "subsystems/OI.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeTeleop :
 public frc2::CommandHelper<frc2::Command, IntakeTeleop> {
 
  public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  IntakeTeleop(std::shared_ptr<Intake> Intake, std::shared_ptr<OI> OI);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

   private:

    std::shared_ptr<Intake> m_intake;
    std::shared_ptr<OI> m_OI;
    bool AButton; // activate
    bool BButton; // deactivate
    bool down;
    bool spinning;
    
    units::angular_velocity::radians_per_second_t WheelAngularVel;
    units::angular_velocity::radians_per_second_t WheelTargetAngularVel;

    units::angular_velocity::radians_per_second_t maximumWheelRotationVelocity = 8.0_rad_per_s;//TODO: get maximum rotation velocity

    units::angular_velocity::radians_per_second_t ArmAngularVel;
    units::angular_velocity::radians_per_second_t ArmTargetAngularVel;

    units::angle::radian_t ArmTargetPosition;
    units::angle::radian_t ArmPosition;
    units::angle::radian_t ArmMaxPosition;
    units::angle::radian_t ArmMinPosition;

};
