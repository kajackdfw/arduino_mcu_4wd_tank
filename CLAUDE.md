# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino-based motor controller for a 4WD tank rover with collision detection and emergency stop capabilities. The Arduino receives JSON commands from a Raspberry Pi (or other SBC) via USB serial (115200 baud) and controls 4 DC motors through 2 L298N motor driver modules.

## Hardware Architecture

**Motors & Drivers:**
- 4x 12V DC motors (one per wheel)
- 2x L298N motor driver modules
  - Left L298N: Controls left track (front + rear motors in parallel)
  - Right L298N: Controls right track (front + rear motors in parallel)

**Safety System:**
- 2x Bumper switches (normally open) - left and right collision detection
- 1x Emergency stop switch (normally closed) - immediate motor cutoff

**Pin Configuration:** See mcu_4wd_tank.ino:3-23 for complete pin definitions
- PWM pins: 3, 5, 6, 9 (motor speeds)
- Digital pins: 22-29 (motor directions)
- Input pins: 10, 11, 12 (safety switches with INPUT_PULLUP)

## Development Commands

### Arduino CLI
```bash
# Compile for Arduino Mega (requires PWM pins 3,5,6,9)
arduino-cli compile --fqbn arduino:avr:mega mcu_4wd_tank.ino

# Upload to board (adjust port as needed)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega mcu_4wd_tank.ino

# Monitor serial output
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

### PlatformIO
```bash
# Build project
pio run

# Upload firmware
pio run --target upload

# Serial monitor
pio device monitor --baud 115200
```

### Arduino IDE
1. Open `mcu_4wd_tank.ino`
2. Select board: Arduino Mega 2560 (or compatible board with sufficient PWM pins)
3. Select port (usually `/dev/ttyACM0` on Linux, `COM3+` on Windows)
4. Install ArduinoJson library (Tools → Manage Libraries → search "ArduinoJson")
5. Upload with Ctrl+U or Upload button

### Testing Commands
```bash
# Send test commands via serial (Linux/Mac)
echo '{"cmd":"forward","speed":150,"duration":2}' > /dev/ttyACM0

# Python test script
python3 << 'EOF'
import serial, json, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(json.dumps({"cmd":"forward","speed":150,"duration":2}).encode() + b'\n')
print(ser.readline().decode())
ser.close()
EOF
```

## Code Architecture

**Main Components:**

1. **Command Parser** (mcu_4wd_tank.ino:102-176)
   - JSON deserialization using ArduinoJson
   - Supports two command formats:
     - Named commands: `{"cmd":"forward", "speed":200}`
     - Direct control: `{"left":200, "right":150}`
   - Optional `duration` parameter for timed movements

2. **Motor Control** (mcu_4wd_tank.ino:179-232)
   - `setTrackSpeeds()`: High-level track control with safety checks
   - `setMotorPair()`: Controls both motors on one L298N driver
   - Speed range: -255 to +255 (negative = reverse)
   - Tank steering: independent left/right track control

3. **Safety System** (mcu_4wd_tank.ino:257-286)
   - `checkSafetySwitches()`: Polls switches every loop iteration
   - Emergency stop: Immediately stops all motors, blocks all commands
   - Collision detection: Triggers automatic recovery sequence
   - Priority: E-stop → Collision → Normal commands

4. **Collision Recovery** (mcu_4wd_tank.ino:288-300)
   - Inverts last movement direction
   - Backs away for `inverse_action_time` (default 1000ms)
   - Sets collision flag requiring `{"cmd":"reset"}` to clear

5. **Timed Movement** (mcu_4wd_tank.ino:92-100, 310-314)
   - Non-blocking timer using `millis()`
   - Automatically stops motors after duration expires
   - Cancellable with stop command

## Command Protocol

**Command Format:** JSON string with newline terminator (`\n`)

**Movement Commands:**
```json
{"cmd":"forward", "speed":200, "duration":2.5}
{"cmd":"backward", "speed":150}
{"cmd":"left", "speed":180, "duration":1}
{"cmd":"right", "speed":180}
{"cmd":"curve", "left_speed":200, "right_speed":150}
{"left":200, "right":150, "duration":2}
```

**Control Commands:**
```json
{"cmd":"stop"}    // Stop motors, cancel timed movement
{"cmd":"reset"}   // Clear collision/emergency flags
```

**Parameters:**
- `speed`: 0-255 (applies to both tracks for forward/backward/pivot)
- `left_speed` / `right_speed`: -255 to 255 (curve command)
- `left` / `right`: -255 to 255 (direct track control)
- `duration`: float seconds (0.5, 1, 2.5, etc.) - optional

**Responses:**
- Success: `"OK"`
- Errors: `"ERROR: Emergency stop active"` / `"ERROR: Collision detected..."`

## Key Implementation Details

**Safety State Machine:**
- `emergencyStop` flag: Set when E-stop pressed, cleared when released
- `leftCollision` / `rightCollision` flags: Set on bumper press, cleared with reset command
- `performingRecovery` flag: True during collision recovery (blocks all commands)

**Non-blocking Timing:**
- Uses `millis()` for all timing (never `delay()`)
- Main loop checks: safety switches → recovery timer → timed movement → serial commands
- Fast loop iteration ensures responsive collision detection

**Motor Control Logic:**
- Forward: Both tracks positive speed
- Backward: Both tracks negative speed
- Pivot left: Left track negative, right track positive
- Pivot right: Left track positive, right track negative
- Curve: Different positive speeds (or mixed positive/negative)

## Dependencies

**Required Libraries:**
- **ArduinoJson** (v6.x) - JSON parsing for commands
  - Install: Arduino IDE → Sketch → Include Library → Manage Libraries → "ArduinoJson"
  - Or: `arduino-cli lib install ArduinoJson@6.21.3`

## Testing Strategy

1. **Bench Test** (wheels off ground):
   - Test each command type with short durations
   - Verify correct wheel rotation directions
   - Test E-stop and collision switches manually

2. **Floor Test** (progressive):
   - Test forward/backward with timed movements
   - Test pivot turns
   - Test collision detection (approach wall slowly)
   - Test E-stop during movement

3. **Integration Test** (with Raspberry Pi):
   - Test Python serial communication
   - Test command sequencing
   - Test error handling and recovery

## Common Modifications

**Adjust Collision Recovery Time:**
```cpp
// Line 35: Change recovery duration
unsigned long inverse_action_time = 1500; // milliseconds
```

**Change Serial Baud Rate:**
```cpp
// Line 48: Update baud rate
Serial.begin(115200); // Must match SBC side
```

**Add Speed Calibration:**
```cpp
// Add near line 196: Scale speeds per track
leftSpeed = map(leftSpeed, -255, 255, -240, 255);  // Left track slightly slower
rightSpeed = map(rightSpeed, -255, 255, -255, 240); // Right track slightly slower
```

**Invert Motor Direction:**
```cpp
// Lines 209-212, 218-221: Swap HIGH/LOW on IN pins if motor runs backwards
digitalWrite(in1Pin, LOW);   // Swap these two
digitalWrite(in2Pin, HIGH);  // to reverse direction
```

## Safety Notes

- E-stop switch must be normally closed (opens when pressed)
- Bumper switches must be normally open (closes when pressed)
- All switches use INPUT_PULLUP (active LOW)
- Motors stop immediately on E-stop (bypasses all safety checks)
- Collision flags persist until explicitly cleared with reset command
- Command timeout is NOT implemented (consider adding for production use)
