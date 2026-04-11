
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterHood.h"
#include "utilities/ShooterTable.h"
#include "subsystems/TargetFinder.h"
#include "subsystems/BallisticShot.h"


/**
 * Continuously update hood position based on automatically ranged shot calculation.
 */
class TrackHood
    : public frc2::CommandHelper<frc2::Command, TrackHood> {
 public:

  TrackHood(std::shared_ptr<ShooterHood>& shooterHood, std::shared_ptr<TargetFinder>& tf, std::shared_ptr<ShooterTable>& st, std::shared_ptr<BallisticShot>& bs, bool shooterTable = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  std::shared_ptr<ShooterHood> m_shooterHood;
  std::shared_ptr<TargetFinder> m_tf;
  std::shared_ptr<ShooterTable> m_st;
  std::shared_ptr<BallisticShot> m_bs;
  bool m_shooterTable;
};
