# WeeWee2026 C++


# CANBus Configuration

This season we're going to leave lower CANBus IDs for system-wide / special
components and start CANBus IDs for subsystem hardware at ID 5.


| Device Name           | CANBusID |   CanBus   |
| --------------------- | -------- | ---------- |
| CANdle LED Control    |    4     |  Canivore  |
| Pigeon2 IMU           |    5     |  Canivore  |
| Swerve 0: Encoder     |    6     |  Canivore  |
| Swerve 0: Drive       |    7     |  Canivore  |
| Swerve 0: Steer       |    8     |  Canivore  |
| Swerve 1: Encoder     |    9     |  Canivore  |
| Swerve 1: Drive       |    10    |  Canivore  |
| Swerve 1: Steer       |    11    |  Canivore  |
| Swerve 2: Encoder     |    12    |  Canivore  |
| Swerve 2: Drive       |    13    |  Canivore  |
| Swerve 2: Steer       |    14    |  Canivore  |
| Swerve 3: Encoder     |    15    |  Canivore  |
| Swerve 3: Drive       |    16    |  Canivore  |
| Swerve 3: Steer       |    17    |  Canivore  |
| Intake Actuator Lead  |    18    |    rio     |
| Intake Actuator Follow|    19    |    rio     |
| Intake Collector Motor|    20    |    rio     |
| Lead Flywheel         |    21    |    rio     |
| Follow Flywheel       |    22    |    rio     |
| Spindexer Motor       |    23    |    rio     |
| Hood Motor            |    24    |    rio     |
| Turret Motor          |    25    |    rio     |
| Turret Encoder        |    26    |    rio     |
| Indexer Motor         |    27    |    rio     |
| Indexer LaserCAN      |    28    |    rio     |
| Climber Motor         |    29    |    rio     |





# OI (Driver Controller)

| Button/Joystick  | Command Name          | Function                      |
| ---------------- | --------------------- | ----------------------------- |
| Left Joystick X  | GetDriverLeftX        | Drivetrain                    |
| Left Joystick Y  | GetDriverLeftY        | Drivetrain                    |
| Right Joystick X | GetDriverRightX       | Drivetrain                    |
| Right Joystick Y | GetDriverRightY       |                               |
| Left Trigger     | GetDriverLeftTrigger  |                               |
| Right Trigger    | GetDriverRightTrigger |                               |
| A Button         | GetDriverAButton      |                               |
| B Button         | GetDriverBButton      |                               |
| X Button         | GetDriverXButton      |                               |
| Y Button         | GetDriverYButton      |                               |
| Menu Button      | GetDriverMenuButton   |                               |
| View Button      | GetDriverViewButton   |                               |
| Left Bumper      | GetDriverLeftBumper   |                               |
| Right Bumper     | GetDriverRightBumper  |                               |
| D-Pad Up         | GetDriverDPadUp       |                               |
| D-Pad Right      | GetDriverDPadRight    |                               |
| D-Pad Left       | GetDriverDPadLeft     |                               |
| D-Pad Down       | GetDriverDPadDown     |                               |
| Zero Controller  | ZeroDriverController  |                               |





# OI (Operator Controller)

| Button/Joystick  | Command Name            | Function                      |
| ---------------- | ----------------------- | ----------------------------- |
| Left Joystick X  | GetOperatorLeftX        | Move Turret Left/Right        |
| Left Joystick Y  | GetOperatorLeftY        |                               |
| Right Joystick X | GetOperatorRightX       |                               |
| Right Joystick Y | GetOperatorRightY       |                               |
| Left Trigger     | GetOperatorLeftTrigger  |                               |
| Right Trigger    | GetOperatorRightTrigger |                               |
| A Button         | GetOperatorAButton      |                               |
| B Button         | GetOperatorBButton      |                               |
| X Button         | GetOperatorXButton      |                               |
| Y Button         | GetOperatorYButton      |                               |
| Menu Button      | GetOperatorMenuButton   |                               |
| View Button      | GetOperatorViewButton   |                               |
| Left Bumper      | GetOperatorLeftBumper   |                               |
| Right Bumper     | GetOperatorRightBumper  |                               |
| D-Pad Up         | GetOperatorDPadUp       |                               |
| D-Pad Right      | GetOperatorDPadRight    |                               |
| D-Pad Left       | GetOperatorDPadLeft     |                               |
| D-Pad Down       | GetOperatorDPadDown     |                               |
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
