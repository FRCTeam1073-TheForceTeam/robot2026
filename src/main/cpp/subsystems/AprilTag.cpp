#include "subsystems/AprilTag.h"

AprilTag::AprilTag() : m_udp_client("0.0.0.0:5800", m_logger) {

  // Implementation of subsystem constructor goes here.
  m_udp_client.start();
}

AprilTag::~AprilTag() {
    m_udp_client.shutdown();
}


void AprilTag::Periodic() {
  // Implementation of subsystem periodic method goes here.
  // TODO:
  auto result = m_udp_client.receive(m_packet_buffer.data(), m_packet_buffer.size());
}

void AprilTag::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
