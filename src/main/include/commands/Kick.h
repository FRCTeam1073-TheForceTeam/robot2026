// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Kicker.h"
#include "subsystems/OI.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Kick
    : public frc2::CommandHelper<frc2::Command, Kick> {
 public:
   /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  Kick(std::shared_ptr<Kicker> kicker, std::shared_ptr<OI> OI);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;




  private:
   std::shared_ptr<Kicker> m_kicker;
   std::shared_ptr<OI> m_OI;
   units::angular_velocity::radians_per_second_t angularVel;
   units::angular_velocity::radians_per_second_t targetAngularVel;
   units::force::newton_t torque;
   static constexpr units::angular_velocity::radians_per_second_t maximumRotationVelocity = 12.34_rad_per_s;//TODO: get maximum rotation velocity
   bool MenuButton;
};
