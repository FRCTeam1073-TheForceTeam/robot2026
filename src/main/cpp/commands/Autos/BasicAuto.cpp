#include "commands/Autos/BasicAuto.h"

frc2::CommandPtr BasicAuto::Create(std::shared_ptr<Drivetrain> drivetrain, std::shared_ptr<Localizer> localizer) {
  // return frc2::InstantCommand(
  //   [drivetrain, localizer]
  //   {drivetrain->SetChassisSpeeds(
  //     frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, localizer->getPose().Rotation())
  //   );}
  // ).ToPtr();
  return DriveStraight(drivetrain, localizer).ToPtr();
}