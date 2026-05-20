# Hood Stall Detection

Automatically cuts power to the Hood motor when it reaches a hard stop (min/max position limits).

## How It Works

When the hood is commanded to move beyond its physical limits (`minPosition = 42°`, `maxPosition = 69°`):

1. Motor hits the hard stop and draws high current
2. Stall counter increments each loop cycle
3. After 15+ consecutive cycles of high current (~150ms), power cuts automatically
4. Motor stops and current drops, counter resets
5. Next command can be sent

This prevents motor damage from sustained stall current.

## Configuration

Located in [ShooterHood.h](ShooterHood.h):

```cpp
static constexpr units::current::ampere_t STALL_CURRENT_THRESHOLD = units::current::ampere_t(18.0);
static constexpr int STALL_COUNT_THRESHOLD = 15; // ~150ms at 100Hz loop rate
```

### STALL_CURRENT_THRESHOLD (18A)
The current above which a stall is suspected. Adjust based on load:
- **Lower (16-18A)** if motor struggles lightly at hard stop
- **Higher (20-24A)** if normal operation draws significant current

### STALL_COUNT_THRESHOLD (15 cycles)
Debounce time to prevent false triggers from momentary current spikes:
- **Lower (10)** for faster response (~100ms)
- **Higher (20-25)** if stall detection triggers too easily

## SmartDashboard Monitoring

During testing, monitor these values on the Driver Station:

| Metric | Purpose |
|--------|---------|
| `Hood/Current` | Live motor current draw (amps) |
| `Hood/Stalled` | Boolean: True when stall detected and power cut |
| `Hood/StallCounter` | Current stall cycle count (0-15) |

## Tuning Procedure

1. **Find stall current**:
   - Manually drive hood forward/backward to each hard stop
   - Watch `Hood/Current` on SmartDashboard
   - Note the peak current value

2. **Set threshold**:
   - Use a value 2-3A below the observed peak
   - Example: If peak is 22A, set threshold to 18-20A

3. **Test with commands**:
   - Send position commands beyond limits
   - Verify `Hood/Stalled` goes true after ~150ms
   - Verify motor stops and doesn't overheat

4. **Adjust if needed**:
   - Too sensitive: Increase `STALL_CURRENT_THRESHOLD` or `STALL_COUNT_THRESHOLD`
   - Not sensitive enough: Decrease either threshold
   - Changes require redeploy to robot

## Implementation Details

See [ShooterHood.cpp:72-86](ShooterHood.cpp#L72) for the stall detection logic:

```cpp
// Stall detection: cut power if motor draws high current with a position command
units::current::ampere_t current = _hoodCurrentSig.GetValue();
bool hasPositionCommand = std::holds_alternative<units::angle::radian_t>(_command);

if (hasPositionCommand && current > STALL_CURRENT_THRESHOLD) {
  _stallCounter++;
  if (_stallCounter >= STALL_COUNT_THRESHOLD) {
    _hoodMotor.SetControl(controls::NeutralOut()); // Cut power
    frc::SmartDashboard::PutBoolean("Hood/Stalled", true);
    return; // Exit early, don't send command this cycle
  }
} else {
  _stallCounter = 0; // Reset if conditions no longer met
  frc::SmartDashboard::PutBoolean("Hood/Stalled", false);
}
```

The `return` statement ensures the motor truly stops by skipping that cycle's control command.

## Replicating to Other Subsystems

To add stall detection to Intake, Climber, or other position-controlled mechanisms:

1. Copy the stall detection constants to the subsystem header
2. Add `int _stallCounter = 0;` member variable
3. Add the stall detection block to `Periodic()` before command processing
4. Adjust current thresholds for that mechanism's load

Example for Intake (lighter load):
```cpp
static constexpr units::current::ampere_t STALL_CURRENT_THRESHOLD = units::current::ampere_t(35.0);
```

Example for Climber (heavy load):
```cpp
static constexpr units::current::ampere_t STALL_CURRENT_THRESHOLD = units::current::ampere_t(50.0);
```
