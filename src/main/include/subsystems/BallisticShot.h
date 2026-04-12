#ifndef _BALLISTIC_SHOT_
#define _BALLISTIC_SHOT_

class TargetFinder;

#include <frc2/command/SubsystemBase.h>
#include "subsystems/TargetFinder.h"

#include<units/velocity.h>
#include<units/angle.h>
#include<units/length.h>
#include<units/acceleration.h>
#include<math.h>
#include<memory>

/**
 * This subsystem continuously updates a cached value for a best-shot using compensated
 * ballistic trajectory.
 */
class BallisticShot : public frc2::SubsystemBase {
 public:
  struct Shot
  {
    units::velocity::meters_per_second_t FlywheelSpeed;
    units::angle::radian_t HoodAngle;
    units::time::second_t ShotTime;
  };
  
  BallisticShot(std::shared_ptr<TargetFinder>& tf);

  // Update the currentstate shot.
  void Periodic() override;

    // Access the currently cached/latest shot value:
  const Shot& GetShot() { return m_currentShot; }

  // Function to recompute shot:
  static Shot ComputeShot(units::length::meter_t range);
  
 private:

  std::shared_ptr<TargetFinder> m_tf;
  Shot m_currentShot;

  static constexpr units::length::meter_t heightAboveHub = 1.0_m;
  static constexpr units::length::meter_t hubHeight = 1.829_m;
  static constexpr units::length::meter_t turretHeight = 0.40_m;
  static constexpr double efficiency = 0.81;   // Calibration for transfer of flywheel velocity to fuel.
  // static constexpr units::angle::radian_t hood_offset = -0.12_rad; // Calibration for shot exit vs. hood angle.
  static constexpr units::angle::radian_t hood_offset = -0.18_rad; // Calibration for shot exit vs. hood angle.

  static constexpr units::acceleration::meters_per_second_squared_t gravity = 9.81_mps_sq;

};

#endif