// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos/WeekZeroAuto.h"

frc2::CommandPtr WeekZeroAuto::Create(
    std::shared_ptr<Spindexer> Spindexer,
    std::shared_ptr<Kicker> Kicker,
    std::shared_ptr<Flywheel> Flywheel,
    std::shared_ptr<ShooterHood> ShooterHood,
    std::shared_ptr<Turret> Turret) {

      // return frc2::SequentialCommandGroup(
      //   frc2::PrintCommand("Inside Sequence"),
      //   frc2::InstantCommand([Flywheel] {Flywheel->SetVelocity(20_mps);}, {Flywheel.get()}),
      //   frc2::InstantCommand([ShooterHood] {ShooterHood->SetCommand(0.1_rad * 3);}, {ShooterHood.get()}), //TODO: find value of scale factor * level
      //   frc2::InstantCommand([Turret] {Turret->SetCommand(0_deg);}, {Turret.get()}),
      //   frc2::WaitCommand(5_s),
      //   frc2::PrintCommand("Post Wait 1"),
      //   frc2::InstantCommand([Spindexer] {Spindexer->SetCommand(1.5_mps);}, {Spindexer.get()}),
      //   frc2::InstantCommand([Kicker] {Kicker->SetCommand(1.65_mps);}, {Kicker.get()}),
      //   frc2::WaitCommand(12_s),
      //   frc2::PrintCommand("Post Wait 2"),
      //   frc2::InstantCommand([Flywheel] {Flywheel->SetVelocity(0_mps);}, {Flywheel.get()}),
      //   frc2::InstantCommand([Spindexer] {Spindexer->SetCommand(0_mps);}, {Spindexer.get()}),
      //   frc2::InstantCommand([Kicker] {Kicker->SetCommand(0_mps);}, {Kicker.get()}),
      //   frc2::PrintCommand("end of command")
      // ).ToPtr();

    // return frc2::cmd::Sequence(
    //   frc2::PrintCommand("Auto A").ToPtr(),
    //   // frc2::WaitCommand(1_s).ToPtr(),
    //   frc2::PrintCommand("Auto B").ToPtr(),
    //   frc2::PrintCommand("Auto C").ToPtr()
    // );


    std::vector<std::unique_ptr<Command>> commands;
    commands.emplace_back(std::make_unique<frc2::InstantCommand>([]{std::cerr << "1st Inst Print" << std::endl;}));
    commands.emplace_back(std::make_unique<frc2::InstantCommand>([]{std::cerr << "2nd Inst Print" << std::endl;}));
    commands.emplace_back(std::make_unique<frc2::PrintCommand>("Auto A"));
    commands.emplace_back(std::make_unique<frc2::WaitCommand>(4_s));
    commands.emplace_back(std::make_unique<frc2::PrintCommand>("Print after 4 seconds"));

    // commands.emplace_back(std::make_unique<frc2::InstantCommand>([Flywheel] {Flywheel->SetVelocity(20_mps);}, {Flywheel.get()}));

    return SequentialCommandGroup(std::move(commands)).ToPtr();

    // return frc2::cmd::Idle(); // A do-nothing command from the commands factory.
  }