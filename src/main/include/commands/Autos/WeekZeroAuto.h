// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>

#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <subsystems/Flywheel.h>
#include <subsystems/Spindexer.h>
#include <subsystems/ShooterHood.h>
#include <subsystems/Kicker.h>
#include <subsystems/Turret.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WeekZeroAuto {
    public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  
    static frc2::CommandPtr Create(
    std::shared_ptr<Spindexer> Spindexer,
    std::shared_ptr<Kicker> Kicker,
    std::shared_ptr<Flywheel> Flywheel,
    std::shared_ptr<ShooterHood> ShooterHood,
    std::shared_ptr<Turret> Turret); 
};
