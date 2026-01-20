# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino-based motor controller for a 4WD tank rover with collision detection, emergency stop capabilities, and **autonomous closed-loop heading control**. The Pro Micro (ATmega32U4) receives JSON commands from a Raspberry Pi (or other SBC) via USB serial (115200 baud) and controls 4 DC motors through 2 L298N motor driver modules. An integrated AK8975 3-axis magnetometer enables autonomous rotation to compass headings.

## Hardware Architecture

**Motors & Drivers:**
- 4x 12V DC motors (one per wheel)
- 2x L298N motor driver modules
  - Left L298N: Controls left track (front + rear motors in parallel)
  - Right L298N: Controls right track (front + rear motors in parallel)

**Navigation:**
- 1x AK8975 3-axis magnetometer (I2C address 0x0C)
  - Provides compass heading (0-360 degrees)
  - Enables autonomous rotation to target headings
  - Hard iron calibration stored in EEPROM

**Safety System:**
- 2x Bumper switches (normally open) - left and right collision detection
- 1x Emergency stop switch (normally closed) - immediate motor cutoff

**Accessories:**
- 1x Relay module (pin 1) - Headlight control (active HIGH)

**Pin Configuration (Pro Micro / ATmega32U4):** See mcu_4wd_tank.ino:3-31 for complete pin definitions
- PWM pins: 5, 6, 9, 10 (motor speeds)
- Digital pins: 4, 7, 8, 14, 15, 16, 18, 19 (motor directions)
- Input pins: 0, 20, 21 (safety switches with INPUT_PULLUP)
- Output pin: 1 (headlight relay control)
- I2C pins: 2 (SDA), 3 (SCL) - Reserved for AK8975

**AK8975 Wiring:**
```
AK8975 VCC  → 5V
AK8975 GND  → GND
AK8975 SDA  → Pro Micro Pin 2 (SDA)
AK8975 SCL  → Pro Micro Pin 3 (SCL)
AK8975 CSB  → 3.3V (enables I2C mode)
AK8975 CAD1 → GND  } I2C address
AK8975 CAD2 → GND  } 0x0C (default)
```

**Headlight Relay Wiring:**
```
Relay Module VCC → 5V
Relay Module GND → GND
Relay Module IN  → Pro Micro Pin 1
Relay COM        → Headlight positive (+)
Relay NO         → Battery/Power positive (+)
Headlight GND    → Battery/Power negative (-)

Note: Use appropriate relay for headlight current (5A+ recommended for LED/halogen)
      Relay activates when Pin 1 is HIGH (headlights ON)
```

## Development Commands

### Arduino CLI
```bash
# Install SparkFun board support (one-time setup)
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json
arduino-cli core update-index
arduino-cli core install SparkFun:avr

# Install required libraries (one-time setup)
arduino-cli lib install ArduinoJson@6.21.3
# Note: I2Cdev and AK8975 must be manually installed from jrowberg/i2cdevlib

# Compile for Pro Micro (ATmega32U4, 5V, 16MHz)
arduino-cli compile --fqbn SparkFun:avr:promicro:cpu=16MHzatmega32U4 mcu_4wd_tank.ino

# Upload to board (adjust port as needed)
arduino-cli upload -p /dev/ttyACM0 --fqbn SparkFun:avr:promicro:cpu=16MHzatmega32U4 mcu_4wd_tank.ino

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
1. Install SparkFun board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json`
   - Tools → Board Manager → search "SparkFun AVR Boards" → Install
2. Install libraries:
   - Tools → Manage Libraries → search "ArduinoJson" → Install v6.x
   - Manual install I2Cdev and AK8975 from https://github.com/jrowberg/i2cdevlib
     - Clone repo, copy `Arduino/I2Cdev` and `Arduino/AK8975` to `~/Arduino/libraries/`
3. Open `mcu_4wd_tank.ino`
4. Select board: SparkFun Pro Micro (Tools → Board → SparkFun AVR Boards → SparkFun Pro Micro)
5. Select processor: ATmega32U4 (5V, 16MHz)
6. Select port (usually `/dev/ttyACM0` on Linux, `COM3+` on Windows)
7. Upload with Ctrl+U or Upload button

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

6. **Heading Control** (mcu_4wd_tank.ino:370-440)
   - `startHeadingControl()`: Initiates autonomous rotation to target heading
   - `performHeadingControl()`: Proportional controller with 3-zone speed control
   - `stopHeadingControl()`: Stops rotation and returns to idle state
   - `calculateHeadingError()`: Computes shortest angular path (handles 0°/360° wraparound)
   - Non-blocking closed-loop control integrated with safety system
   - 30-second timeout protection prevents infinite loops

7. **Compass Functions** (mcu_4wd_tank.ino:442-530)
   - `readCompassHeading()`: Reads AK8975, applies calibration, returns 0-360° heading
   - `loadCalibration()`: Loads hard iron offsets from EEPROM on startup
   - `saveCalibration()`: Stores calibration data to EEPROM
   - `performCompassCalibration()`: Auto-calibration by rotating robot 360°
   - 20 Hz compass update rate during heading control, 10 Hz when idle

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
{"cmd":"stop"}    // Stop motors, cancel timed movement and heading control
{"cmd":"reset"}   // Clear collision/emergency flags
```

**Headlight Commands:**
```json
{"cmd":"headlights_on"}      // Turn headlights ON
{"cmd":"headlights_off"}     // Turn headlights OFF
{"cmd":"headlights_toggle"}  // Toggle headlights (ON ↔ OFF)
```

**Heading Control Commands:**
```json
{"cmd":"rotate_to_heading", "heading":90, "speed":180, "tolerance":5}
// Autonomously rotate to absolute heading (0-359°)
// heading: 0=North, 90=East, 180=South, 270=West
// speed: max rotation speed 40-255 (default: 180)
// tolerance: acceptable error in degrees (default: 5)

{"cmd":"rotate_by", "degrees":90, "speed":180}
// Rotate by relative angle (+90 = right 90°, -90 = left 90°)
// Internally converts to absolute heading

{"cmd":"compass"}
// Query current heading and raw magnetometer values
// Response: {"heading":142.5,"raw_x":245,"raw_y":-120,"raw_z":15}

{"cmd":"calibrate_compass", "duration":10}
// Auto-calibrate compass by rotating 360° for specified duration (seconds)
// Calculates hard iron offsets and stores in EEPROM
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
- `headingControlActive` flag: True during autonomous heading control

**Priority Hierarchy:**
1. Emergency Stop (highest) → Cancels all operations, stops motors
2. Collision Recovery → Cancels heading control, runs recovery sequence
3. **Heading Control** → Autonomous closed-loop navigation
4. Timed Movement → Scheduled motor operations
5. Serial Commands (lowest)

**Non-blocking Timing:**
- Uses `millis()` for all timing (never `delay()`)
- Main loop checks: compass update → safety switches → collision recovery → heading control → timed movement → serial commands
- Fast loop iteration (~5ms typical, ~40ms with compass read) ensures responsive control

**Heading Control Algorithm:**
- **Zone 1** (|error| > 30°): Full speed rotation (fastest approach)
- **Zone 2** (10° < |error| ≤ 30°): Proportional speed 50-100% (smooth deceleration)
- **Zone 3** (tolerance < |error| ≤ 10°): Minimum speed 40% (precision alignment)
- Automatically chooses shortest rotation path (handles 359°→0° wraparound)
- 30-second timeout prevents deadlock if target unreachable

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

- **I2Cdev** - I2C device library (dependency for AK8975)
  - Install: Clone https://github.com/jrowberg/i2cdevlib
  - Copy `Arduino/I2Cdev` folder to `~/Arduino/libraries/`

- **AK8975** - 3-axis magnetometer driver
  - Install: Clone https://github.com/jrowberg/i2cdevlib
  - Copy `Arduino/AK8975` folder to `~/Arduino/libraries/`

- **Wire** - I2C communication (built-in to Arduino core)
- **EEPROM** - Non-volatile storage for calibration (built-in to Arduino core)

## Testing Strategy

1. **Bench Test** (wheels off ground):
   - Test each command type with short durations
   - Verify correct wheel rotation directions
   - Test E-stop and collision switches manually
   - **Compass Test**: Send `{"cmd":"compass"}` while manually rotating robot - verify heading changes 0°→90°→180°→270°→0°
   - **Rotation Direction**: Send `{"cmd":"rotate_to_heading","heading":90,"speed":100}` - verify correct rotation direction

2. **Calibration** (first-time setup):
   - Place robot on floor in area free of magnetic interference
   - Send `{"cmd":"calibrate_compass","duration":10}`
   - Robot rotates slowly for 10 seconds, calculates offsets, stores to EEPROM
   - Reboot and verify calibration persists

3. **Floor Test** (progressive):
   - **Headlights**: Test `{"cmd":"headlights_on"}`, `{"cmd":"headlights_off"}`, `{"cmd":"headlights_toggle"}`
   - Test forward/backward with timed movements
   - Test pivot turns
   - **Heading Control**: Send `{"cmd":"rotate_to_heading","heading":0}` then 90, 180, 270 - verify ±5° accuracy
   - **Relative Rotation**: Send `{"cmd":"rotate_by","degrees":90}` - verify 90° turn
   - **Wraparound**: From ~350° heading, send `{"cmd":"rotate_to_heading","heading":10}` - should rotate CW ~20°, not CCW 340°
   - Test collision detection (approach wall slowly)
   - Test E-stop during movement and heading control

4. **Integration Test** (with Raspberry Pi):
   - Test Python serial communication
   - Test command sequencing
   - Test error handling and recovery
   - **Autonomous Navigation**: Sequence of heading commands for waypoint navigation

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

## Migration from Arduino Mega to Pro Micro

**Critical Changes:**
- Pin 3 moved from PWM motor control to I2C SCL (required for AK8975)
- PWM pins remapped: {3,5,6,9} → {5,6,9,10}
- Direction pins remapped from D22-29 → {4,7,8,14,15,16,18,19}
- Safety switches remapped: D10,11,12 → D20,21,0
- Added pin 1 for headlight relay control
- Board configuration: `SparkFun:avr:promicro:cpu=16MHzatmega32U4`

**Pin Budget:**
- 18 pins available on Pro Micro
- 18 pins used (4 PWM + 8 direction + 3 safety + 1 headlight + 2 I2C)
- **All pins fully utilized**

**Memory Footprint:**
- Flash: ~10 KB / 28 KB (36% used) - Comfortable headroom for future features
- SRAM: ~720 bytes / 2560 bytes (28% used) - Sufficient for all operations
- EEPROM: 8 bytes used (calibration data at address 0)

**Performance:**
- Loop time: ~5ms typical, ~7ms with compass read (20 Hz control loop)
- Compass update: 50ms during heading control (20 Hz), 100ms idle (10 Hz)
- I2C clock: 400 kHz (fast mode) for minimal latency

## Safety Notes

- E-stop switch must be normally closed (opens when pressed)
- Bumper switches must be normally open (closes when pressed)
- All switches use INPUT_PULLUP (active LOW)
- Motors stop immediately on E-stop (bypasses all safety checks)
- Collision flags persist until explicitly cleared with reset command
- Heading control automatically cancelled on E-stop or collision
- 30-second timeout prevents infinite heading control loops
- Command timeout is NOT implemented (consider adding for production use)
- **Compass calibration recommended**: Run calibration after first boot and whenever magnetic environment changes
