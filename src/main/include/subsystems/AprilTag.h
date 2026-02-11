

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <wpinet/UDPClient.h>
#include <wpi/Logger.h>
#include <array>

class AprilTag : public frc2::SubsystemBase {
 public:
  AprilTag();

  ~AprilTag();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  wpi::Logger m_logger;
  wpi::UDPClient m_udp_client;
  std::array<uint8_t, 1024> m_packet_buffer;
};