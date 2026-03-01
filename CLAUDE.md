# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino-based motor controller for a 4WD tank rover with collision detection, emergency stop capabilities, and wheel odometry. The target board is a **Pro Micro Board Module ATmega32U4 5V 16MHz** (Arduino IDE compatible). It receives JSON commands from a Raspberry Pi (or other SBC) via USB serial (115200 baud) and controls 4 DC motors through 2 L298N motor driver modules.

## Hardware Architecture

**Motors & Drivers:**
- 4x 12V DC motors (one per wheel)
- 2x L298N motor driver modules
  - Left L298N: Controls left track (front + rear motors in parallel)
  - Right L298N: Controls right track (front + rear motors in parallel)

**Safety System:**
- 2x Bumper switches (normally open) - left and right collision detection
- 1x Emergency stop switch (normally closed) - immediate motor cutoff

**Wheel Odometry:**
- 2x FET sensors (one per track) triggered by magnets on wheels
- Pulse counting via hardware interrupts (INT0 / INT1) - no pulses missed regardless of loop timing
- Used for drive speed calibration

**Pin Configuration (Pro Micro Board Module ATmega32U4 5V 16MHz):** See mcu_4wd_tank.ino:3-30 for complete pin definitions
- PWM pins: 5, 6, 9, 10 (motor speeds)
- Digital pins: 4, 7, 8, 14, 15, 16, 18, 19 (motor directions)
- Input pins: 0, 20, 21 (safety switches with INPUT_PULLUP)
- Interrupt pins: 2 (INT0, left wheel sensor), 3 (INT1, right wheel sensor)

**Wheel Sensor Wiring:**
```
FET Sensor VCC → 5V (or 3.3V per sensor spec)
FET Sensor GND → GND
FET Sensor OUT → Pro Micro Pin 2 (left) / Pin 3 (right)

Note: Pins use INPUT_PULLUP. Interrupts trigger on RISING edge.
      Change to FALLING in attachInterrupt() calls if sensor pulls LOW on trigger.
      One magnet per wheel gives one pulse per revolution.
      Multiple magnets increase resolution (pulses per revolution).
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

1. **Command Parser** (mcu_4wd_tank.ino:98-172)
   - JSON deserialization using ArduinoJson
   - Supports two command formats:
     - Named commands: `{"cmd":"forward", "speed":200}`
     - Direct control: `{"left":200, "right":150}`
   - Optional `duration` parameter for timed movements

2. **Motor Control** (mcu_4wd_tank.ino:175-215)
   - `setTrackSpeeds()`: High-level track control with safety checks
   - `setMotorPair()`: Controls both motors on one L298N driver
   - Speed range: -255 to +255 (negative = reverse)
   - Tank steering: independent left/right track control

3. **Safety System** (mcu_4wd_tank.ino:233-270)
   - `checkSafetySwitches()`: Polls switches every loop iteration
   - Emergency stop: Immediately stops all motors, blocks all commands
   - Collision detection: Triggers automatic recovery sequence
   - Priority: E-stop → Collision → Normal commands

4. **Collision Recovery** (mcu_4wd_tank.ino:272-283)
   - Inverts last movement direction
   - Backs away for `inverse_action_time` (default 1000ms)
   - Sets collision flag requiring `{"cmd":"reset"}` to clear

5. **Wheel Odometry** (mcu_4wd_tank.ino:~295-310)
   - `leftWheelISR()` / `rightWheelISR()`: Hardware interrupt handlers, increment `volatile` pulse counters
   - Attached to INT0 (pin 2) and INT1 (pin 3) via `attachInterrupt()`
   - `noInterrupts()` / `interrupts()` guards used when reading counters from main loop
   - Counters persist across movements; reset explicitly with `reset_odometry` command

6. **Timed Movement** (mcu_4wd_tank.ino:88-95, 285-289)
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

**special curve macro**
``` these are curve command preset values, based on manual calibration they are mapped to The command buttons marked with rewind or fast forward icons that are curve around 22.5, 45, and 90. these will use some preset values for left and right motor speeds. The presets values will be based on some calculations based on a dictionary of rover gemometry settings. In code we will call them 
1. curve_FL_90 ( forward and left curve to 90 degree ccw )
1. curve_FL_45 ( forward and left curve to 45 degree ccw )
1. curve_FL_22_5 ( forward and left curve to 22.5 degree ccw )
1. curve_FR_90 ( forward and right curve 90 degree cw )
1. curve_FR_45 ( forward and right curve to 45 degree cw )
1. curve_FR_22_5 ( forward and right curve to 22.5 degree cw )
1. curve_RL_90 ( reverse and left curve to 90 degree cw )
1. curve_RL_45 ( reverse and left curve to 45 degree cw )
1. curve_RL_22_5 ( reverse and left curve to 22.5 degree cw )
1. curve_RR_90 ( reverse and right curve 90 degree ccw )
1. curve_RR_45 ( reverse and right curve to 45 degree ccw )
1. curve_RR_22_5 ( reverse and right curve to 22.5 degree ccw )
These will all use a speed setting called autonimous_speed, instead of default_speed. Using a dictionary for each, a drive command of {"cmd":"curve", "left_speed":200, "right_speed":150, "distance": 420} . The odometer sensor on the outer wheel will be used for determining distance traveled. The inner wheel odometer will be recorded and compared at the end of the maneuver to a stored value to determine accuracy of the curve manuever and reported to the driver. Different terrains may evoke wheel slippage and may be a subject to analyze and adjust for in the future. 


**Control Commands:**
```json
{"cmd":"stop"}             // Stop motors, cancel timed movement
{"cmd":"reset"}            // Clear collision/emergency flags
{"cmd":"odometry"}         // Query wheel pulse counts
{"cmd":"reset_odometry"}   // Zero both pulse counters
```

**Odometry Response:**
```json
{"left_pulses":142,"right_pulses":139}
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

**Priority Hierarchy:**
1. Emergency Stop (highest) → Stops motors immediately
2. Collision Recovery → Runs recovery sequence
3. Timed Movement → Scheduled motor operations
4. Serial Commands (lowest)

**Non-blocking Timing:**
- Uses `millis()` for all timing (never `delay()`)
- Main loop checks: safety switches → collision recovery → timed movement → serial commands
- Fast loop iteration ensures responsive control

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
   - **Odometry**: Send `{"cmd":"odometry"}` while spinning wheels by hand - verify counts increment
   - **Sensor polarity**: Confirm pulse counts increase; if not, change `RISING` to `FALLING` in `attachInterrupt()` calls

2. **Floor Test** (progressive):
   - Test forward/backward with timed movements
   - Test pivot turns
   - **Odometry**: Send `{"cmd":"reset_odometry"}`, drive a known distance, query `{"cmd":"odometry"}` - record pulses per metre for each track
   - Test collision detection (approach wall slowly)
   - Test E-stop during movement

3. **Integration Test** (with Raspberry Pi):
   - Test Python serial communication
   - Test command sequencing
   - Test error handling and recovery
   - **Speed calibration**: Use odometry data to characterise left/right track speed differences

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
// In setTrackSpeeds(): Scale speeds per track
leftSpeed = map(leftSpeed, -255, 255, -240, 255);  // Left track slightly slower
rightSpeed = map(rightSpeed, -255, 255, -255, 240); // Right track slightly slower
```

**Invert Motor Direction:**
```cpp
// In setMotorPair(): Swap HIGH/LOW on IN pins if motor runs backwards
digitalWrite(in1Pin, LOW);   // Swap these two
digitalWrite(in2Pin, HIGH);  // to reverse direction
```

## Board Reference

**Target Board:** Pro Micro Board Module ATmega32U4 5V 16MHz (Arduino IDE compatible)
**Board configuration:** `SparkFun:avr:promicro:cpu=16MHzatmega32U4`

**Full Pin Allocation:**
| Pin | Function     | Type |
|-----|--------------|------|
| 0   | E-stop switch| Input (INPUT_PULLUP) |
| 1   | *unused*     | - |
| 2   | Left wheel sensor (INT0) | Input (INPUT_PULLUP, interrupt) |
| 3   | Right wheel sensor (INT1) | Input (INPUT_PULLUP, interrupt) |
| 4   | LEFT_IN1     | Output |
| 5   | LEFT_ENA     | PWM Output |
| 6   | LEFT_ENB     | PWM Output |
| 7   | LEFT_IN2     | Output |
| 8   | LEFT_IN3     | Output |
| 9   | RIGHT_ENA    | PWM Output |
| 10  | RIGHT_ENB    | PWM Output |
| 14  | LEFT_IN4     | Output |
| 15  | RIGHT_IN1    | Output |
| 16  | RIGHT_IN2    | Output |
| 18  | RIGHT_IN3    | Output |
| 19  | RIGHT_IN4    | Output |
| 20  | Left bumper  | Input (INPUT_PULLUP) |
| 21  | Right bumper | Input (INPUT_PULLUP) |

**Pin Budget:**
- 18 pins available on Pro Micro
- 17 pins used (4 PWM + 8 direction + 3 safety + 2 wheel sensors)
- 1 pin free (pin 1)

**Memory Footprint:**
- Flash: ~6 KB / 28 KB (~21% used) - Ample headroom for future features
- SRAM: ~500 bytes / 2560 bytes (~20% used) - Sufficient for all operations

**Performance:**
- Loop time: ~1ms typical - fast, responsive control

## Safety Notes

- E-stop switch must be normally closed (opens when pressed)
- Bumper switches must be normally open (closes when pressed)
- All switches use INPUT_PULLUP (active LOW)
- Motors stop immediately on E-stop (bypasses all safety checks)
- Collision flags persist until explicitly cleared with reset command
- Command timeout is NOT implemented (consider adding for production use)
