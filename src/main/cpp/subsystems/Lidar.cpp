#include "subsystems/Lidar.h"
#include <iostream>
#include <thread>

#include <frc/Timer.h>


Lidar::Lidar() :
    _port(460800, frc::SerialPort::Port::kUSB1, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One),
    _hardwareConfigured(false),
    _measureMode(false)
{
    _port.SetReadBufferSize(4096);
    _hardwareConfigured = ConfigureHardware();
    if (!_hardwareConfigured) {
        std::cerr << "Lidar Hardware not configured!" << std::endl;
    }

    // Up to 250 scans without reallocation.
    for (int ii = 0; ii < _feedback.scans.size(); ++ii) {
        _feedback.scans[ii] = Scan();
        _workingFeedback.scans[ii] = Scan();
    }
    _feedback.valid_scans = 0;
    _workingFeedback.valid_scans = 0;

    frc::SmartDashboard::PutBoolean("Lidar/Lidar - hardware_configured", _hardwareConfigured);
}

void Lidar::Periodic() {

    if (_hardwareConfigured) {
        // Try to process scans and update feedback.
        auto numBytes = _port.GetBytesReceived();

        if (_measureMode) {
            // Reading measurements and parsing / updating feedback.
            ReadAndParseMeasurements(numBytes);
        } else if (numBytes >= 7) {
            // Waiting to read  header / acknowledge.
            if (ParseDescriptor()) {
                _measureMode = true; // Next read is data...
            } else {
                std::cerr << "Lidar handshake error" << std::endl;
            }
        }

    } else {
        // Make sure we say feedback is invalid in this case.
        _feedback.valid_scans = 0;
        _workingFeedback.valid_scans = 0;
    }
}


                // double startTime =  Timer.getFPGATimestamp();
                // // Check bytes available and run forever in loop....
                // int numBytesAvail = serialPort.getBytesReceived();
                
                // if (measureMode){
                //     // read and parse all available scan data to read - fill array
                //     readAndParseMeasurements(numBytesAvail);
                // } else if (numBytesAvail >= 7) {
                //     // read and check the first 7 bytes of response
                //     if(parseDescriptor()){
                //         // expected descriptor received, switch to read data
                //         measureMode = true;
                //     }
                //     else{
                //         System.out.println("LidarIO: Lidar handshake error");
                //     }
                // }
                // double endTime = Timer.getFPGATimestamp();
                // // Measure how we are doing on each iteration:
                // setLastIterationTime(endTime - startTime);



            //             public void readAndParseMeasurements(int numBytesAvail){
            // // round down to determine number of full scans available
            // int numScansToRead = numBytesAvail/bytesPerScan;
            // byte[] rawData = serialPort.read(numScansToRead * bytesPerScan);
            // for(int i = 0; i < numScansToRead; i ++) {
            //     int offset = i * bytesPerScan;
            //     boolean recordScan = true; // Starts as true.
            //     if ((rawData[offset] & 0x003) == 1) {

            //         // If we have enough, then process scan and update output.
            //         if (readScanCount > 20) {
            //             // This method has a synchronized block to store data for external threads to access.
            //             calculateOutput(readScanCount, Timer.getFPGATimestamp()); // Updates outputs based on scan we have right now.
            //         }
            //         // Always start over accumulating new data for next time:
            //         readScanCount = 0;
            //     } 
                
            //     // divide by 4 to drop the lower two bits
            //     int quality = ((rawData[offset + 0] & 0x0FC) >> 2);
            //     if (quality < minAcceptedQuality) recordScan = false;
            //     // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
            //     float angle_deg = ((Byte.toUnsignedInt(rawData[offset + 2]) & 0x0FF) << 7) | ((Byte.toUnsignedInt(rawData[offset + 1]) & 0x0FE) >> 1);
            //     angle_deg /= 64.0f + mountingOffset;
            //     // range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
            //     float range_mm = ((rawData[offset + 4] & 0x0FF) << 8) | (rawData[offset + 3] & 0x0FF);
            //     float angle_rad = 3.141592f * angle_deg / 180.0f;
            //     float range_m = (range_mm / 4.0f) / 1000;

            //     // Cartesian point in lidar coordinates:
            //     double x_l = Math.cos(-angle_rad) * range_m;
            //     double y_l = Math.sin(-angle_rad) * range_m;
               
            //     // Cartesian endpoint in robot coordinates:
            //     double x_robot = x_l + x_offset;
            //     double y_robot = y_l + y_offset;

            //     // Quality, range, and angle filter
            //     if (isAngleGood(angle_deg) == false) recordScan = false;
            //     if (isRangeGood(range_m) == false) recordScan = false;
            //     if (!isXInRange(x_robot)) recordScan = false;
            //     if (!isYInRange(y_robot)) recordScan = false;

            //     // If the scan is good, then record it:
            //     if(recordScan) {
            //         // If we have space to record it:
            //         if (readScanCount < 250) {
            //             var scan = scans.get(readScanCount);
            //             // Fill out existing scan:
            //             scan.range = range_m;
            //             scan.angle = angle_rad;
            //             scan.quality = quality;
            //             scan.x_robot = x_robot;
            //             scan.y_robot = y_robot;
            //             readScanCount++; // Count this scan for array one.
            //         }
            //     } // Else we don't record this one... it gets filtered out.
            // }

void Lidar::ReadAndParseMeasurements(int numBytes) {

    std::array<char, bytesPerScan * 256> buffer; // Make sure buffer is large enough.

    // Figure out how many scans we can read?
    int numScansToRead = numBytes / bytesPerScan; // Round down to ignore partial scans:
    if (numScansToRead > 256) numScansToRead = 256; // Safety bound.
    auto read_bytes = _port.Read(buffer.data(), numScansToRead * bytesPerScan); // Read as many scans as we can.

    if (read_bytes != numScansToRead * bytesPerScan) {
        std::cerr << "Lidar: Mismatched Read!!" << std::endl;
    }

    // Loop over the binary data and parse it for these scans:
    for (int ii(0); ii < numScansToRead; ++ii) {
        const int offset = ii * bytesPerScan; // Byte offset of this scan record in the data.
        bool recordScan = true; // Assume this one is good until we think it isn't.

        // Parse data:
        // divide by 4 to drop the lower two bits
        unsigned int qbyte = static_cast<unsigned int>(buffer[offset]);
        unsigned int quality = (qbyte & 0x0FC) >> 2;
        if (quality < minQuality) recordScan = false;

        unsigned int hi_angle = static_cast<unsigned int>(buffer[offset + 2]);
        unsigned int lo_angle = static_cast<unsigned int>(buffer[offset + 1]);
        unsigned int angle_deg = ((hi_angle & 0x0FF) << 7) | ((lo_angle & 0x0FE) >> 1);
        float angle_degrees = angle_deg / 64.0f;

        // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
        // float angle_deg = ((Byte.toUnsignedInt(buffer[offset + 2]) & 0x0FF) << 7) | ((Byte.toUnsignedInt(buffer[offset + 1]) & 0x0FE) >> 1);
        // angle_deg /= 64.0f + mountingOffset;

        unsigned int range_hi = static_cast<unsigned int>(buffer[offset + 4]);
        unsigned int range_lo = static_cast<unsigned int>(buffer[offset + 3]);
        unsigned int range_mm = ((range_hi & 0x0FF) << 8) | ((range_lo & 0x0FF));

        if (range_mm > maxRangeMM) recordScan = false;

        // range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
        // float range_mm = ((buffer[offset + 4] & 0x0FF) << 8) | (buffer[offset + 3] & 0x0FF);
        // float angle_rad = 3.141592f * angle_deg / 180.0f;
        // float range_m = (range_mm / 4.0f) / 1000;

        // Include good data into our working feedback:
        if (recordScan) {
            const int index = _workingFeedback.valid_scans;
            _workingFeedback.scans[index].range = units::length::millimeter_t(range_mm);
            _workingFeedback.scans[index].angle = units::angle::degree_t(angle_degrees);
            _workingFeedback.scans[index].quality = quality;

            // Point in lidar coordinates:
            frc::Translation2d lidar_point = frc::Translation2d(_workingFeedback.scans[index].range, 
                frc::Rotation2d(_workingFeedback.scans[index].angle));
            
            // Transform by lidar transformation:
            auto robot_point = lidar_point.RotateBy(lidarTransform.Rotation());
            robot_point = robot_point + lidarTransform.Translation();

            _workingFeedback.scans[index].robot_point = robot_point;
            _workingFeedback.valid_scans++; // Move to the next scan position to record it.
        }

        // Is this scan the last scan in its group? Then update our output feedback.
        if ((buffer[offset] & 0x003) == 1) {

            // If we have enough, then process scan and update output.
            if (_workingFeedback.valid_scans > 10) {
                // Output this data:
                _feedback.scans = _workingFeedback.scans; // Copy the scans.
                _feedback.valid_scans = _workingFeedback.valid_scans;
                frc::SmartDashboard::PutNumber("Lidar/Angle0(r)", _feedback.scans[0].angle.value()); // Let's us know lidar is doing ok.
                frc::SmartDashboard::PutNumber("Lidar/Range0(m)", _feedback.scans[0].range.value()); // Let's us know lidar is doing ok.
            } else {
                _feedback.valid_scans = 0; // Hit end of scan with less than 10 good points?
            }
            _feedback.start_time = _workingFeedback.start_time;
            _feedback.end_time = frc::Timer::GetFPGATimestamp();

            frc::SmartDashboard::PutNumber("Lidar/ValidScans", _feedback.valid_scans); // Let's us know lidar is doing ok.

            // Always start over accumulating new data for next scan.
            _workingFeedback.start_time = _feedback.end_time; // Time for the next...
            _workingFeedback.valid_scans = 0;
        }
    } // End loop over reading scans.
}


bool Lidar::ConfigureHardware() {
    _port.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
    _port.SetFlowControl(frc::SerialPort::FlowControl::kFlowControl_None);
    std::cerr << "Lidar: handshaking..." << std::endl;
    bool result = Handshake();
    std::cerr << "Lidar: handshake done." << std::endl;
    return result;
}

bool Lidar::ParseDescriptor() {
    std::array<char, 7> buffer;
    _port.Read(buffer.data(), buffer.size()); // Read exactly 7 bytes.
    return buffer == scanDescriptor;
}

bool Lidar::Handshake() {
    // Send Stop
    _port.Write(stopCommand.data(), stopCommand.size());
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait a moment...
    _port.Flush();
    _port.Reset(); // Clear out the data.

    // startCommand
    _port.Write(startCommand.data(), startCommand.size());
    std::cerr << "Lidar: Sent start command to Lidar sensor" << std::endl;

    // wait loop, while getBytesReceived is less, sleep a little.
    int counter  = 0;
    while(_port.GetBytesReceived() < 7 && counter < 15){
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
        counter ++;
    }

    // We didn't hear back...
    if (counter >= 15) {
        return false;
    }

    _measureMode = false;
    return true;
}
