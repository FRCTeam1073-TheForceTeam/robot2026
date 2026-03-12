# WeeWee2026 C++

# WARNING: Fieldmaps
At each competition you must:
1. Update the fieldmap loaded in fieldmap
2. Update target positions in target finder subsystem
3. Update zones in zone finder subsystem
4. Upload correct field .json to both vision servers

# Robot Network IPs

|       Location       |     IP     | Cameras on Server |
| -------------------- | ---------- | ----------------- |
| Radio                | 10.10.73.1 |       N/A         |
| RoboRio              | 10.10.73.2 |       N/A         |
| 1st Vision Server    | 10.10.73.3 |   FR, FL, BR, BL  |
| 2nd Vision Server    | 10.10.73.7 | Turret, Collector |
| Home AP (Plank)      | 10.10.73.4 |       N/A         |
| Driver Station (FMS) | 10.10.73.5 |       N/A         |

# CANBus Configuration

This season we're going to leave lower CANBus IDs for system-wide / special
components and start CANBus IDs for subsystem hardware at ID 5.


| Device Name                    | CANBusID |   CanBus   |
| ------------------------------ | -------- | ---------- |
| CANdle LED Control             |    4     |  Canivore  |
| Pigeon2 IMU                    |    5     |  Canivore  |
| Swerve 0: Encoder              |    6     |  Canivore  |
| Swerve 0: Drive                |    7     |  Canivore  |
| Swerve 0: Steer                |    8     |  Canivore  |
| Swerve 1: Encoder              |    9     |  Canivore  |
| Swerve 1: Drive                |    10    |  Canivore  |
| Swerve 1: Steer                |    11    |  Canivore  |
| Swerve 2: Encoder              |    12    |  Canivore  |
| Swerve 2: Drive                |    13    |  Canivore  |
| Swerve 2: Steer                |    14    |  Canivore  |
| Swerve 3: Encoder              |    15    |  Canivore  |
| Swerve 3: Drive                |    16    |  Canivore  |
| Swerve 3: Steer                |    17    |  Canivore  |
| Intake Actuator Lead (Left)    |    18    |    rio     |
| Intake Actuator Follow (Right) |    19    |    rio     |
| Intake Collector Motor         |    20    |    rio     |
| Lead Flywheel                  |    21    |    rio     |
| Follow Flywheel                |    22    |    rio     |
| Spindexer Motor                |    23    |    rio     |
| Hood Motor                     |    24    |    rio     |
| Turret Motor                   |    25    |    rio     |
| Turret Encoder                 |    26    |    rio     |
| Kicker Motor                   |    27    |    rio     |
| Indexer LaserCAN               |    28    |    rio     |
| Climber Motor                  |    29    |  Canivore  |





# OI (Driver Controller)

| Button/Joystick  | Command Name          | Function                      |
| ---------------- | --------------------- | ----------------------------- |
| Left Joystick X  | GetDriverLeftX        | Drivetrain                    |
| Left Joystick Y  | GetDriverLeftY        | Drivetrain                    |
| Right Joystick X | GetDriverRightX       | Drivetrain                    |
| Right Joystick Y | GetDriverRightY       |                               |
| Left Trigger     | GetDriverLeftTrigger  | Collecter Reverse             |
| Right Trigger    | GetDriverRightTrigger | Collector Spin                |
| A Button         | GetDriverAButton      |                               |
| B Button         | GetDriverBButton      |                               |
| X Button         | GetDriverXButton      |                               |
| Y Button         | GetDriverYButton      | Slow Mode                     |
| Menu Button      | GetDriverMenuButton   | Climber Up                    |
| View Button      | GetDriverViewButton   | Climber Down                  |
| Left Bumper      | GetDriverLeftBumper   | Field Centric                 |
| Right Bumper     | GetDriverRightBumper  | Intake In/Out.                |
| D-Pad Up         | GetDriverDPadUp       |                               |
| D-Pad Right      | GetDriverDPadRight    |                               |
| D-Pad Left       | GetDriverDPadLeft     |                               |
| D-Pad Down       | GetDriverDPadDown     |                               |
| Zero Controller  | ZeroDriverController  |                               |






# OI (Operator Controller)

| Button/Joystick  | Command Name            | Function                      |
| ---------------- | ----------------------- | ----------------------------- |
| Left Joystick X  | GetOperatorLeftX        | Moves Turret                  |
| Left Joystick Y  | GetOperatorLeftY        |                               |
| Right Joystick X | GetOperatorRightX       |                               |
| Right Joystick Y | GetOperatorRightY       |                               |
| Left Trigger     | GetOperatorLeftTrigger  | Auto Aim Flywheel and Turret  |
| Right Trigger    | GetOperatorRightTrigger | Shoot, Spindexer + Kicker foward|
| A Button         | GetOperatorAButton      |                               |
| B Button         | GetOperatorBButton      | Spindexer + Kicker Backwards  |
| X Button         | GetOperatorXButton      | Tower Set Shot                |
| Y Button         | GetOperatorYButton      | Corner Set Shot               |
| Menu Button      | GetOperatorMenuButton   |                               |
| View Button      | GetOperatorViewButton   |                               |
| Left Bumper      | GetOperatorLeftBumper   |                               |
| Right Bumper     | GetOperatorRightBumper  |                               |
| D-Pad Up         | GetOperatorDPadUp       | Zero Turret                   |
| D-Pad Right      | GetOperatorDPadRight    | Zero Hood                     |
| D-Pad Left       | GetOperatorDPadLeft     | Zero Intake                   |
| D-Pad Down       | GetOperatorDPadDown     | Zero Climber                  |
| Zero Controller  | ZeroOperatorController  |                               |





Add mechanisms after this...
| Device Name        | CANBusID | CanBus     |
| ------------------ | -------- | ---------- |
| Elevator Front     |    18    |  rio       |
| Spindexer          |    19    |  rio       |
| Feeder             |    20    |  rio       |
| Turret             |    21    |  rio       |
| ________           |    22    |  rio       |
| ________           |    23    |  rio       |
| ________           |    24    |  rio       |
| ________           |    25    |  rio       |
| ________           |    26    |  rio       |
| ________           |    27    |  rio       |

# System Network Addresses

- Vision Server 1 : 10.10.73.3
- AP Radio: 10.10.73.4
- Vision Server 2: 10.10.73.7
- Robo Rio: 10.10.73.2
- Robot Radio: 10.10.73.1




