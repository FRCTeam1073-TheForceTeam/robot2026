// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/WeekZeroAuto.h"



frc2::CommandPtr WeekZeroAuto::Create(
    std::shared_ptr<Spindexer> Spindexer,
    std::shared_ptr<Kicker> Kicker,
    std::shared_ptr<Flywheel> Flywheel,
    std::shared_ptr<ShooterHood> ShooterHood,
    std::shared_ptr<Turret> Turret){
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(
          frc2::InstantCommand(std::function<void()>(Flywheel->SetCommand(20_mps)), {&Flywheel}),
          frc2::InstantCommand([] {ShooterHood->SetCommand(-0.1_rad * 3);}, {&ShooterHood}), //TODO: find value of scale factor * level
          frc2::InstantCommand([] {Turret->SetCommand(0_deg);}, {&Turret}),
        ),
        frc2::WaitCommand(1_s),
        frc2::ParallelCommandGroup(
          frc2::InstantCommand([] {Spindexer->SetCommand(1.5_mps);}, {&Spindexer}),
          frc2::InstantCommand([] {Kicker->SetCommand(1.65_mps);}, {&Kicker}),
        ),
        frc2::WaitCommand(12_s),
        frc2::ParallelCommandGroup(
          frc2::InstantCommand([] {Flywheel->SetCommand(0_mps);}, {&Flywheel}),
          frc2::InstantCommand([] {Spindexer->SetCommand(0_mps);}, {&Spindexer}),
          frc2::InstantCommand([] {Kicker->SetCommand(0_mps);}, {&Kicker}),
        )
      ).ToPtr();
    }