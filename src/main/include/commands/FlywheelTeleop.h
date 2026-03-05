// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Flywheel.h"
#include "subsystems/TargetFinder.h"
#include "utilities/ShooterTable.h"
#include "subsystems/OI.h"
#include <units/angular_velocity.h>

/**
 * Flywheel Teleop command.
 */
class FlywheelTeleop
    : public frc2::CommandHelper<frc2::Command, FlywheelTeleop> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param flywheel The subsystem used by this command.
   */
  explicit FlywheelTeleop(std::shared_ptr<Flywheel>& flywheel, std::shared_ptr<OI>& oi, std::shared_ptr<TargetFinder>& hf, std::shared_ptr<ShooterTable>& st);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  
 private:
  std::shared_ptr<Flywheel> m_flywheel;
  std::shared_ptr<OI> m_OI;
  std::shared_ptr<TargetFinder> m_hf;
  std::shared_ptr<ShooterTable> m_st;

  bool XButton;
  bool DPadDown;
  bool DPadUp;

  bool LastDPadUpState;
  bool LastDPadDownState;

  double scale;

  short level;
  const units::velocity::meters_per_second_t scaleFactor = 0.16666_mps; //TODO: change value
  const short maxLevel = 120; //TODO: change value
  
  units::velocity::meters_per_second_t maxVel;
};
