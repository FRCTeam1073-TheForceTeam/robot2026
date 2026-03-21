#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Turret.h"
#include <subsystems/TargetFinder.h>
#include <subsystems/AprilTagFinder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LockTurret
    : public frc2::CommandHelper<frc2::Command, LockTurret> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  LockTurret(std::shared_ptr<Turret>& turret, std::shared_ptr<TargetFinder>& targetFinder, std::shared_ptr<AprilTagFinder>& aprilTag);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private:

  bool isAlignedToHub;

  double lastError;

  std::shared_ptr<Turret> m_turret;
  std::shared_ptr<TargetFinder> m_targetFinder;
  std::shared_ptr<AprilTagFinder> m_aprilTag;


  units::angle::radian_t m_targetPosition;
  units::angle::radian_t position;//zeroed position is up against the hard stop

};
