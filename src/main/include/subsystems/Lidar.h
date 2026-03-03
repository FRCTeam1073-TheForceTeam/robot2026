#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SerialPort.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>


#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <array>


/**
 * This system provides lidar feedback for alignment.
 * 
 */
class Lidar : public frc2::SubsystemBase {
 public:

    // Each lidar scan:
    class Scan {
        public:
        //sampling mode once receives express scan request - get_lidar_conf (command to get typical scan mode)
        //quality, angle, angle, distance, distance
        //actual heading = angle_q6/64.0 degree
        //actual distance = distance_q2/4.0 mm
        units::length::meter_t range;
        units::angle::radian_t angle;
        float quality; // Arbitrary scale of quality.
        frc::Translation2d robot_point; // Point in robot coordinates.

        Scan() : range(0_m), angle(0_rad), quality(0), robot_point(frc::Translation2d()) {};

        Scan(units::length::meter_t r, units::angle::radian_t a, float q, 
                    const frc::Translation2d & point) {
            range = r;
            angle = a;
            quality = q;
            robot_point = point;
        }

        // std::string toString(){
        //     return "Range: " + range + " Angle: " + angle + " Quality: " + quality.to_string();
        // }
    };

    // Data packet parsing values and commands:
    static constexpr std::array<char, 2> getInfo = { 0xa5, 0x52};
    static constexpr std::array<char, 7> scanDescriptor = {0xa5, 0x5a, 0x05, 0x00, 0x00, 0x40, 0x81};
    static constexpr std::array<char, 2> stopCommand = { 0xa5, 0x25};
    static constexpr std::array<char, 2> startCommand = { 0xa5, 0x20};
    static constexpr unsigned int bytesPerScan = 5;

    // Location of Lidar in robot:
    static constexpr frc::Transform2d lidarTransform = {0.0_m, 0.0_m, frc::Rotation2d(0_deg)};

    // Thresholds on values:
    static constexpr float minQuality = 5;
    static constexpr unsigned int maxRangeMM = 2000; // 2m
  
  // The feedback for this subsystem provided as a struct.
  struct Feedback {
    Feedback() : start_time(0_s), end_time(0_s),valid_scans(0) {};
    units::time::second_t start_time; // When this scan was started.
    units::time::second_t end_time;
    unsigned int valid_scans;
    std::array<Scan, 256> scans; // Vector of individual scans.
  };

  // Constructor for the subsystem.
  Lidar();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   * This function samples and updates feedback from hardware, and sends target
   * command information to the hardware.
   */
  void Periodic() override;


  /// Access the latest feedback from the system. 
  const Feedback& GetFeedback() const { return _feedback; }

 private:

  // Helper function for configuring hardware from within the constructor of the subsystem.
  bool ConfigureHardware();
  
  // Serialport:
  frc::SerialPort _port;

  // Handshake message to start LIDAR running.
  bool Handshake();

  // Read back descriptor to make sure we can start reading data.
  bool ParseDescriptor();

  // Given the number of bytes, read and parse the measurement data.
  void ReadAndParseMeasurements(int numBytes);

  // Did we successfully configure the hardware?
  bool _hardwareConfigured;

  // False while we're waiting for the header, true if we reading scans.
  bool _measureMode; 
  
  // Cached feedback:
  Feedback _feedback;
  Feedback _workingFeedback;


};
