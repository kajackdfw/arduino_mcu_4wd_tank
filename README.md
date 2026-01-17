# 4WD Tank Rover - Complete Documentation

## Overview
This project implements a 4-wheel drive tank-style rover controlled by an Arduino, which receives commands from a Raspberry Pi via USB Serial connection. The rover features tank steering, collision detection, emergency stop capability, and automatic collision recovery.

## Hardware Components

### Motors & Controllers
- **4x 12V DC Motors** - One per wheel
- **2x L298N Motor Drivers**
  - Left L298N: Controls both left side motors (front + rear)
  - Right L298N: Controls both right side motors (front + rear)

### Microcontrollers
- **Arduino** - Motor control and safety monitoring
- **Raspberry Pi** - High-level control, connected to Arduino via USB

### Safety Sensors
- **2x Bumper Switches** (normally open)
  - Left bumper for left side collision detection
  - Right bumper for right side collision detection
- **1x Emergency Stop Switch** (normally closed)
  - Opens circuit when pressed to immediately stop all motors

## Pin Configuration

### Arduino Pins

#### Left Track L298N
| Pin | Function | Description |
|-----|----------|-------------|
| 3 | LEFT_ENA | Left front motor speed (PWM) |
| 22 | LEFT_IN1 | Left front motor direction |
| 23 | LEFT_IN2 | Left front motor direction |
| 5 | LEFT_ENB | Left rear motor speed (PWM) |
| 24 | LEFT_IN3 | Left rear motor direction |
| 25 | LEFT_IN4 | Left rear motor direction |

#### Right Track L298N
| Pin | Function | Description |
|-----|----------|-------------|
| 6 | RIGHT_ENA | Right front motor speed (PWM) |
| 26 | RIGHT_IN1 | Right front motor direction |
| 27 | RIGHT_IN2 | Right front motor direction |
| 9 | RIGHT_ENB | Right rear motor speed (PWM) |
| 28 | RIGHT_IN3 | Right rear motor direction |
| 29 | RIGHT_IN4 | Right rear motor direction |

#### Safety Switches
| Pin | Function | Type |
|-----|----------|------|
| 10 | LEFT_BUMPER | Normally open (INPUT_PULLUP) |
| 11 | RIGHT_BUMPER | Normally open (INPUT_PULLUP) |
| 12 | ESTOP_SWITCH | Normally closed (INPUT_PULLUP) |

## Wiring Notes
- All safety switches connect between their pin and GND
- Internal pullup resistors are enabled on all switch pins
- Bumpers: LOW when pressed (collision detected)
- E-Stop: LOW when pressed (emergency stop activated)

## Command Protocol

### Communication
- **Interface**: USB Serial (Arduino ↔ Raspberry Pi)
- **Baud Rate**: 115200
- **Format**: JSON commands sent as strings with newline terminator
- **Responses**: Arduino sends "OK" or error messages

### Movement Commands

#### Forward
```json
{"cmd":"forward", "speed":200}
{"cmd":"forward", "speed":200, "duration":2.5}
```
- Moves both tracks forward at the same speed
- Speed range: 0-255
- Optional duration in seconds

#### Backward
```json
{"cmd":"backward", "speed":200}
{"cmd":"backward", "speed":150, "duration":1.5}
```
- Moves both tracks backward at the same speed
- Speed range: 0-255
- Optional duration in seconds

#### Pivot Left
```json
{"cmd":"left", "speed":180}
{"cmd":"left", "speed":180, "duration":1}
```
- Left track moves backward, right track moves forward
- Creates sharp pivot turn in place
- Speed range: 0-255
- Optional duration in seconds

#### Pivot Right
```json
{"cmd":"right", "speed":180}
{"cmd":"right", "speed":180, "duration":1}
```
- Right track moves backward, left track moves forward
- Creates sharp pivot turn in place
- Speed range: 0-255
- Optional duration in seconds

#### Curve Turn
```json
{"cmd":"curve", "left_speed":200, "right_speed":150}
{"cmd":"curve", "left_speed":200, "right_speed":150, "duration":3}
```
- Independent speed control for each track
- Creates smooth curved paths
- Speed range: -255 to 255 (negative = reverse)
- Default speeds: 150 if not specified
- Optional duration in seconds

#### Direct Track Control
```json
{"left":200, "right":150}
{"left":200, "right":150, "duration":2}
```
- Direct control of left and right track speeds
- Most flexible control method
- Speed range: -255 to 255 (negative = reverse)
- Optional duration in seconds

### Control Commands

#### Stop
```json
{"cmd":"stop"}
```
- Immediately stops all motors
- Cancels any active timed movement
- Does NOT clear collision flags

#### Reset
```json
{"cmd":"reset"}
```
- Clears collision detection flags
- Resets emergency stop flag
- Required after collision to resume normal operation

## Duration Parameter
All movement commands support an optional `"duration"` parameter:
- Specified in **seconds** (float)
- Motors run for specified time then automatically stop
- Can be cancelled early with stop command
- Examples: `0.5`, `1`, `2.5`, `10`

**Without duration**: Motors run continuously until stop command or collision

**With duration**: Motors run for specified time then stop automatically

## Safety Features

### Emergency Stop
- **Trigger**: E-stop switch pressed (circuit opens)
- **Action**: Immediately stops all motors
- **Status**: "EMERGENCY STOP ACTIVATED" printed to serial
- **Recovery**: Release e-stop switch, send `{"cmd":"reset"}` to clear

### Collision Detection
- **Trigger**: Bumper switch pressed (normally open closes)
- **Action**: 
  1. Immediately reverses last movement direction
  2. Backs away for `inverse_action_time` (default 1000ms)
  3. Stops motors
  4. Sets collision flag
- **Messages**: 
  - "LEFT COLLISION DETECTED - Reversing"
  - "RIGHT COLLISION DETECTED - Reversing"
  - "Recovery complete"
- **Recovery**: Send `{"cmd":"reset"}` to clear collision flags

### Safety Priority
1. **E-stop** - Highest priority, stops everything
2. **Collision Detection** - Triggers automatic recovery
3. **Movement Commands** - Blocked if safety flags are set

### Configurable Parameters
```cpp
unsigned long inverse_action_time = 1000; // Collision recovery duration in milliseconds
```

## Behavior & Logic

### Tank Steering
- **Left Track**: Front left motor + Rear left motor (move in unison)
- **Right Track**: Front right motor + Rear right motor (move in unison)
- Independent track control enables:
  - Forward/backward movement (both tracks same speed/direction)
  - Pivot turns (tracks opposite directions)
  - Curved turns (tracks different speeds, same direction)
  - Point turns (one track stopped, other moving)

### Collision Recovery Behavior
When collision detected:
1. **Capture** current movement direction and speed
2. **Reverse** the movement (invert speeds)
3. **Execute** reverse movement for `inverse_action_time` duration
4. **Stop** motors after recovery complete
5. **Set** collision flag (requires reset to resume)

Examples:
- Moving forward at 200 → backs up at -200 for 1 second
- Turning right (left:200, right:-100) → reverses to (left:-200, right:100)
- Stopped (0 speed) → no movement during recovery

### Timed Movement Behavior
When duration specified:
1. **Start** motors at requested speed
2. **Track** elapsed time
3. **Stop** automatically when duration expires
4. **Print** "Timed movement complete" to serial
5. **Allow** new commands to override current timed movement

## Required Libraries
- **ArduinoJson** (version 6.x)
  - Install via Arduino IDE: Sketch → Include Library → Manage Libraries
  - Search for "ArduinoJson" by Benoit Blanchon

## Serial Communication Examples

### Basic Movement Sequence
```
Send: {"cmd":"forward","speed":200,"duration":2}
Response: OK
(After 2 seconds): Timed movement complete

Send: {"cmd":"right","speed":180,"duration":1}
Response: OK
(After 1 second): Timed movement complete

Send: {"cmd":"stop"}
Response: OK
```

### Collision Scenario
```
Send: {"cmd":"forward","speed":200}
Response: OK

(Collision occurs)
Output: RIGHT COLLISION DETECTED - Reversing
(Motors reverse for 1 second)
Output: Recovery complete

Send: {"cmd":"forward","speed":200}
Response: ERROR: Collision detected, send {"cmd":"reset"} to clear

Send: {"cmd":"reset"}
Response: Safety flags reset

Send: {"cmd":"forward","speed":200}
Response: OK
```

### Emergency Stop Scenario
```
Send: {"cmd":"forward","speed":200}
Response: OK

(E-stop pressed)
Output: EMERGENCY STOP ACTIVATED

Send: {"cmd":"forward","speed":200}
Response: ERROR: Emergency stop active

(E-stop released)
Output: Emergency stop cleared

Send: {"cmd":"reset"}
Response: Safety flags reset

Send: {"cmd":"forward","speed":200}
Response: OK
```

## Testing Commands

### Basic Movement Tests
```json
{"cmd":"forward","speed":150,"duration":2}
{"cmd":"backward","speed":150,"duration":2}
{"cmd":"left","speed":180,"duration":1}
{"cmd":"right","speed":180,"duration":1}
{"cmd":"stop"}
```

### Curve Tests
```json
{"cmd":"curve","left_speed":200,"right_speed":100,"duration":2}
{"cmd":"curve","left_speed":100,"right_speed":200,"duration":2}
{"cmd":"curve","left_speed":200,"right_speed":-50,"duration":1}
```

### Direct Control Tests
```json
{"left":200,"right":200,"duration":2}
{"left":150,"right":200,"duration":2}
{"left":-150,"right":-150,"duration":1}
{"left":0,"right":180,"duration":1}
```

### Safety Tests
```json
// Test collision recovery (manually trigger bumper)
{"cmd":"forward","speed":200}
// Press bumper, observe reversal
{"cmd":"reset"}

// Test e-stop
{"cmd":"forward","speed":200}
// Press e-stop, motors should stop
// Release e-stop
{"cmd":"reset"}
{"cmd":"forward","speed":200}
```

## Raspberry Pi Integration

### Python Example (Serial Communication)
```python
import serial
import json
import time

# Open serial connection to Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

def send_command(cmd_dict):
    """Send JSON command to Arduino"""
    cmd_json = json.dumps(cmd_dict) + '\n'
    ser.write(cmd_json.encode())
    response = ser.readline().decode().strip()
    print(f"Sent: {cmd_json.strip()} | Response: {response}")
    return response

# Example usage
send_command({"cmd": "forward", "speed": 200, "duration": 2})
time.sleep(2.5)

send_command({"cmd": "right", "speed": 180, "duration": 1})
time.sleep(1.5)

send_command({"cmd": "stop"})

# Close connection
ser.close()
```

### Finding Arduino Serial Port
**Linux/Raspberry Pi:**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
# Usually /dev/ttyACM0 or /dev/ttyUSB0
```

**macOS:**
```bash
ls /dev/tty.usb*
# Usually /dev/tty.usbmodemXXXX
```

**Windows:**
```
Check Device Manager → Ports (COM & LPT)
Usually COM3, COM4, etc.
```

## Troubleshooting

### Motors Don't Move
1. Check e-stop switch is not pressed
2. Verify collision flags cleared with `{"cmd":"reset"}`
3. Check L298N power connections (12V supply)
4. Verify all motor driver enable pins connected
5. Test with simple command: `{"cmd":"forward","speed":100}`

### Serial Communication Issues
1. Verify baud rate is 115200
2. Check USB cable connection
3. Ensure correct serial port selected
4. Add newline character to commands
5. Check for JSON syntax errors

### Collision Recovery Not Working
1. Verify bumper switches connected to correct pins
2. Test switch continuity with multimeter
3. Check that rover was moving when collision occurred (recovery inverts last movement)
4. Adjust `inverse_action_time` if backing up too little/much

### One Side Not Moving
1. Check connections for that specific L298N
2. Verify both ENA and ENB pins connected and PWM-capable
3. Test individual track with direct control: `{"left":200,"right":0}`
4. Check motor power supply connections

### Timed Movements Not Stopping
1. Ensure proper JSON format with newline
2. Check Arduino not resetting during operation
3. Verify `millis()` functioning (shouldn't overflow for 50 days)

## Power Considerations

### Motor Power
- L298N requires separate power for motors (typically 7-12V)
- Ensure adequate current capacity for 4 motors
- Recommended: 12V battery pack with 5A+ capacity
- Use common ground between Arduino and motor power supply

### Arduino Power
- Can be powered via USB from Raspberry Pi
- Or use separate 5V regulator from battery
- Ensure stable power during motor operation

## Future Enhancements

### Potential Additions
- [ ] Ultrasonic distance sensors for obstacle avoidance
- [ ] IMU/Gyroscope for heading control
- [ ] GPS module for outdoor navigation
- [ ] Camera integration with Raspberry Pi
- [ ] Battery voltage monitoring
- [ ] Current sensing for motor load detection
- [ ] PID control for precise movements
- [ ] Odometry/encoder support for accurate distance measurement
- [ ] Multiple speed profiles (slow/medium/fast presets)
- [ ] Acceleration/deceleration ramps for smooth starts/stops

### Software Improvements
- [ ] Add motor speed calibration values
- [ ] Implement command queue for sequential movements
- [ ] Add telemetry reporting (position, heading, battery)
- [ ] Create higher-level navigation commands
- [ ] Add diagnostic mode for testing individual components

## License & Usage
This documentation and associated Arduino firmware are provided as-is for educational and hobbyist use. Modify and adapt as needed for your specific rover configuration.

## Version History
- **v1.0** - Initial implementation with tank steering
- **v1.1** - Added collision detection and recovery
- **v1.2** - Added timed movement duration parameter
- **v1.3** - Optimized to use 2 L298N drivers instead of 4
- **v1.4** - Added curve command for smooth turns

---

**Happy Roving! �
