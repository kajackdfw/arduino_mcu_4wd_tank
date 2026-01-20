#include <ArduinoJson.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <AK8975.h>
#include <EEPROM.h>

// Motor Pin Definitions (Pro Micro / ATmega32U4)
// Left Track L298N (controlling Motors 1 & 3)
#define LEFT_ENA 5    // Left front motor speed (PWM)
#define LEFT_IN1 4    // Left front motor direction
#define LEFT_IN2 7    // Left front motor direction
#define LEFT_ENB 6    // Left rear motor speed (PWM)
#define LEFT_IN3 8    // Left rear motor direction
#define LEFT_IN4 14   // Left rear motor direction

// Right Track L298N (controlling Motors 2 & 4)
#define RIGHT_ENA 9    // Right front motor speed (PWM)
#define RIGHT_IN1 15   // Right front motor direction
#define RIGHT_IN2 16   // Right front motor direction
#define RIGHT_ENB 10   // Right rear motor speed (PWM)
#define RIGHT_IN3 18   // Right rear motor direction
#define RIGHT_IN4 19   // Right rear motor direction

// Safety Switches
#define LEFT_BUMPER 20    // Left collision sensor (normally open)
#define RIGHT_BUMPER 21   // Right collision sensor (normally open)
#define ESTOP_SWITCH 0    // Emergency stop (normally closed)

// Headlight Relay
#define HEADLIGHT_RELAY 1 // Relay control for headlights (HIGH = ON)

// I2C Pins (Reserved for AK8975 Magnetometer)
// Pin 2: SDA (I2C Data)
// Pin 3: SCL (I2C Clock)

// Safety state
bool emergencyStop = false;
bool leftCollision = false;
bool rightCollision = false;

// Headlight state
bool headlightsOn = false;

// Current track speeds (for collision recovery)
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// Collision recovery timing
unsigned long inverse_action_time = 1000; // milliseconds
bool performingRecovery = false;
unsigned long recoveryStartTime = 0;

// Timed movement control
bool timedMovement = false;
unsigned long movementStartTime = 0;
unsigned long movementDuration = 0;

// JSON buffer
StaticJsonDocument<256> doc;

// Magnetometer
AK8975 mag;
int16_t rawMagX, rawMagY, rawMagZ;
float currentHeading = 0.0;
bool compassInitialized = false;

// Heading control state
bool headingControlActive = false;
float targetHeading = 0.0;
float headingTolerance = 5.0;
int maxRotationSpeed = 180;
unsigned long headingControlStartTime = 0;
const unsigned long HEADING_CONTROL_TIMEOUT = 30000; // 30 seconds

// Compass timing
unsigned long lastCompassRead = 0;
unsigned long compassReadInterval = 50;  // 50ms = 20 Hz during heading control
bool compassDataReady = false;

// Calibration
bool compassCalibrated = false;
int16_t offsetX = 0, offsetY = 0, offsetZ = 0;
const int EEPROM_CALIB_ADDR = 0;
const uint16_t CALIB_MAGIC = 0xC0AB;

// Control parameters
const int DEFAULT_ROTATION_SPEED = 180;
const float DEFAULT_HEADING_TOLERANCE = 5.0;
const float MIN_HEADING_TOLERANCE = 2.0;
const float MAX_HEADING_TOLERANCE = 20.0;

void setup() {
  Serial.begin(115200);

  // Initialize I2C for compass (400 kHz fast mode)
  Wire.begin();
  Wire.setClock(400000);

  // Initialize AK8975 magnetometer
  Serial.print("Initializing AK8975...");
  mag.initialize();
  compassInitialized = mag.testConnection();

  if (compassInitialized) {
    Serial.println("OK");
    loadCalibration();  // Load from EEPROM if available
  } else {
    Serial.println("FAILED - compass unavailable");
  }

  // Initialize left track L298N pins as outputs
  pinMode(LEFT_ENA, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENB, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  
  // Initialize right track L298N pins as outputs
  pinMode(RIGHT_ENA, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  
  // Initialize safety switches as inputs with pullup resistors
  pinMode(LEFT_BUMPER, INPUT_PULLUP);
  pinMode(RIGHT_BUMPER, INPUT_PULLUP);
  pinMode(ESTOP_SWITCH, INPUT_PULLUP);

  // Initialize headlight relay as output (start OFF)
  pinMode(HEADLIGHT_RELAY, OUTPUT);
  digitalWrite(HEADLIGHT_RELAY, LOW);
  headlightsOn = false;

  // Start with motors stopped
  stopAllMotors();

  Serial.println("Tank Rover Ready");
}

void loop() {
  // Non-blocking compass update
  if (millis() - lastCompassRead >= compassReadInterval) {
    if (compassInitialized) {
      currentHeading = readCompassHeading();
      compassDataReady = true;
      lastCompassRead = millis();
    }
  }

  // Check safety switches first
  checkSafetySwitches();

  // Handle collision recovery (highest priority after safety)
  if (performingRecovery) {
    if (millis() - recoveryStartTime >= inverse_action_time) {
      // Recovery complete, stop motors
      performingRecovery = false;
      forceStopMotors(); // Direct stop, bypassing safety checks
      Serial.println("Recovery complete");
    }
    return; // Don't process commands during recovery
  }

  // Heading control (higher priority than timed movement)
  if (headingControlActive && compassDataReady) {
    performHeadingControl();
    compassDataReady = false;
    // Don't return - allow serial processing below
  }

  // Handle timed movement
  if (timedMovement) {
    if (millis() - movementStartTime >= movementDuration) {
      // Movement duration complete, stop motors
      timedMovement = false;
      stopAllMotors();
      Serial.println("Timed movement complete");
    }
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    DeserializationError error = deserializeJson(doc, input);
    
    if (error) {
      Serial.print("JSON Parse Error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Check for reset command to clear collision flags
    if (doc.containsKey("cmd") && doc["cmd"] == "reset") {
      leftCollision = false;
      rightCollision = false;
      emergencyStop = false;
      Serial.println("Safety flags reset");
      return;
    }
    
    // Get duration parameter if present (in seconds, convert to milliseconds)
    unsigned long duration = 0;
    if (doc.containsKey("duration")) {
      float durationSec = doc["duration"];
      duration = (unsigned long)(durationSec * 1000);
    }
    
    // Check for direct track control
    if (doc.containsKey("left") && doc.containsKey("right")) {
      int leftSpeed = doc["left"];
      int rightSpeed = doc["right"];
      setTrackSpeeds(leftSpeed, rightSpeed);
      
      if (duration > 0) {
        startTimedMovement(duration);
      }
      
      Serial.println("OK");
      return;
    }
    
    // Check for command-based control
    if (doc.containsKey("cmd")) {
      String cmd = doc["cmd"].as<String>();
      int speed = doc.containsKey("speed") ? doc["speed"] : 200;
      
      if (cmd == "forward") {
        setTrackSpeeds(speed, speed);
      } else if (cmd == "backward") {
        setTrackSpeeds(-speed, -speed);
      } else if (cmd == "left") {
        setTrackSpeeds(-speed, speed);  // Left track back, right forward
      } else if (cmd == "right") {
        setTrackSpeeds(speed, -speed);  // Right track back, left forward
      } else if (cmd == "curve") {
        // Curve command with independent track speeds
        int leftSpeed = doc.containsKey("left_speed") ? doc["left_speed"] : 150;
        int rightSpeed = doc.containsKey("right_speed") ? doc["right_speed"] : 150;
        setTrackSpeeds(leftSpeed, rightSpeed);
      } else if (cmd == "stop") {
        stopAllMotors();
        timedMovement = false; // Cancel any timed movement
        headingControlActive = false; // Cancel heading control
      } else if (cmd == "rotate_to_heading") {
        if (!compassInitialized) {
          Serial.println("ERROR: Compass not initialized");
          return;
        }
        float heading = doc.containsKey("heading") ? doc["heading"] : 0.0;
        int speedVal = doc.containsKey("speed") ? doc["speed"] : DEFAULT_ROTATION_SPEED;
        float tolerance = doc.containsKey("tolerance") ? doc["tolerance"] : DEFAULT_HEADING_TOLERANCE;

        // Validate parameters
        heading = constrain(heading, 0, 359);
        tolerance = constrain(tolerance, MIN_HEADING_TOLERANCE, MAX_HEADING_TOLERANCE);

        startHeadingControl(heading, speedVal, tolerance);
        Serial.print("OK - Target heading: ");
        Serial.println(heading);
        return;
      } else if (cmd == "rotate_by") {
        if (!compassInitialized) {
          Serial.println("ERROR: Compass not initialized");
          return;
        }
        float degrees = doc.containsKey("degrees") ? doc["degrees"] : 0.0;
        int speedVal = doc.containsKey("speed") ? doc["speed"] : DEFAULT_ROTATION_SPEED;
        float tolerance = doc.containsKey("tolerance") ? doc["tolerance"] : DEFAULT_HEADING_TOLERANCE;

        // Calculate absolute target from current + relative
        float target = currentHeading + degrees;
        if (target < 0) target += 360;
        if (target >= 360) target -= 360;

        startHeadingControl(target, speedVal, tolerance);
        Serial.print("OK - Rotating ");
        Serial.print(degrees);
        Serial.println(" degrees");
        return;
      } else if (cmd == "compass") {
        if (!compassInitialized) {
          Serial.println("ERROR: Compass not initialized");
          return;
        }
        Serial.print("{\"heading\":");
        Serial.print(currentHeading);
        Serial.print(",\"raw_x\":");
        Serial.print(rawMagX);
        Serial.print(",\"raw_y\":");
        Serial.print(rawMagY);
        Serial.print(",\"raw_z\":");
        Serial.print(rawMagZ);
        Serial.println("}");
        return;
      } else if (cmd == "calibrate_compass") {
        if (!compassInitialized) {
          Serial.println("ERROR: Compass not initialized");
          return;
        }
        int durationVal = doc.containsKey("duration") ? doc["duration"] : 10;
        performCompassCalibration(durationVal);
        return;
      } else if (cmd == "headlights_on") {
        digitalWrite(HEADLIGHT_RELAY, HIGH);
        headlightsOn = true;
        Serial.println("OK - Headlights ON");
        return;
      } else if (cmd == "headlights_off") {
        digitalWrite(HEADLIGHT_RELAY, LOW);
        headlightsOn = false;
        Serial.println("OK - Headlights OFF");
        return;
      } else if (cmd == "headlights_toggle") {
        headlightsOn = !headlightsOn;
        digitalWrite(HEADLIGHT_RELAY, headlightsOn ? HIGH : LOW);
        Serial.print("OK - Headlights ");
        Serial.println(headlightsOn ? "ON" : "OFF");
        return;
      } else {
        Serial.println("Unknown command");
        return;
      }
      
      if (duration > 0) {
        startTimedMovement(duration);
      }
      
      Serial.println("OK");
    }
  }
}

void setTrackSpeeds(int leftSpeed, int rightSpeed) {
  // Check if movement is allowed
  if (emergencyStop) {
    Serial.println("ERROR: Emergency stop active");
    return;
  }
  
  if (leftCollision || rightCollision) {
    Serial.println("ERROR: Collision detected, send {\"cmd\":\"reset\"} to clear");
    return;
  }
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Store current speeds for collision recovery
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  
  // Set left track (both motors on left L298N)
  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, leftSpeed);
  
  // Set right track (both motors on right L298N)
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, rightSpeed);
}

void setMotorPair(int enaPin, int in1Pin, int in2Pin, int enbPin, int in3Pin, int in4Pin, int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
    analogWrite(enaPin, speed);
    analogWrite(enbPin, speed);
  } else if (speed < 0) {
    // Backward
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
    analogWrite(enaPin, abs(speed));
    analogWrite(enbPin, abs(speed));
  } else {
    // Stop
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW);
    analogWrite(enaPin, 0);
    analogWrite(enbPin, 0);
  }
}

void setMotor(int enablePin, int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enablePin, speed);
  } else if (speed < 0) {
    // Backward
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enablePin, abs(speed));
  } else {
    // Stop
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enablePin, 0);
  }
}

void stopAllMotors() {
  setTrackSpeeds(0, 0);
}

void checkSafetySwitches() {
  // Check emergency stop (normally closed, LOW when pressed)
  if (digitalRead(ESTOP_SWITCH) == LOW) {
    if (!emergencyStop) {
      emergencyStop = true;
      headingControlActive = false;  // Cancel heading control
      forceStopMotors();
      Serial.println("EMERGENCY STOP ACTIVATED");
    }
    return; // Don't check other switches if E-stop is active
  } else {
    if (emergencyStop) {
      emergencyStop = false;
      Serial.println("Emergency stop cleared");
    }
  }
  
  // Check left bumper (normally open, LOW when pressed)
  if (digitalRead(LEFT_BUMPER) == LOW && !leftCollision) {
    leftCollision = true;
    Serial.println("LEFT COLLISION DETECTED - Reversing");
    performCollisionRecovery();
  }
  
  // Check right bumper (normally open, LOW when pressed)
  if (digitalRead(RIGHT_BUMPER) == LOW && !rightCollision) {
    rightCollision = true;
    Serial.println("RIGHT COLLISION DETECTED - Reversing");
    performCollisionRecovery();
  }
}

void performCollisionRecovery() {
  // Cancel heading control
  if (headingControlActive) {
    headingControlActive = false;
    Serial.println("Heading control cancelled - collision detected");
  }

  // Calculate inverse speeds
  int inverseLeft = -currentLeftSpeed;
  int inverseRight = -currentRightSpeed;

  // Apply inverse action directly (bypass safety checks)
  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, inverseLeft);
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, inverseRight);

  // Start recovery timer
  performingRecovery = true;
  recoveryStartTime = millis();
}

void forceStopMotors() {
  // Direct motor stop, bypassing all safety checks
  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, 0);
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, 0);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

void startTimedMovement(unsigned long duration) {
  timedMovement = true;
  movementStartTime = millis();
  movementDuration = duration;
}

// ===== HEADING CONTROL FUNCTIONS =====

void startHeadingControl(float target, int speed, float tolerance) {
  if (!compassInitialized) return;

  if (!compassCalibrated) {
    Serial.println("WARNING: Compass not calibrated, accuracy may be reduced");
  }

  targetHeading = target;
  maxRotationSpeed = constrain(speed, 40, 255);
  headingTolerance = tolerance;
  headingControlActive = true;
  headingControlStartTime = millis();

  // Set faster compass read rate during heading control
  compassReadInterval = 50;  // 20 Hz
}

void performHeadingControl() {
  // Timeout check
  if (millis() - headingControlStartTime >= HEADING_CONTROL_TIMEOUT) {
    stopHeadingControl();
    Serial.println("ERROR: Heading control timeout - target not reached");
    return;
  }

  // Calculate heading error (shortest angular distance)
  float error = calculateHeadingError(currentHeading, targetHeading);

  // Check if within tolerance
  if (abs(error) <= headingTolerance) {
    stopHeadingControl();
    Serial.print("Heading reached: ");
    Serial.println(currentHeading);
    return;
  }

  // Proportional control with 3 zones
  int motorSpeed;

  if (abs(error) > 30) {
    // Zone 1: Full speed
    motorSpeed = maxRotationSpeed;
  } else if (abs(error) > 10) {
    // Zone 2: Proportional speed (50-100%)
    float speedScale = map(abs(error), 10, 30, 50, 100) / 100.0;
    motorSpeed = (int)(maxRotationSpeed * speedScale);
  } else {
    // Zone 3: Minimum speed for precision
    motorSpeed = max(40, (int)(maxRotationSpeed * 0.4));
  }

  // Constrain to safe range
  motorSpeed = constrain(motorSpeed, 40, 255);

  // Determine rotation direction (error < 0 = CCW, error > 0 = CW)
  if (error < 0) {
    // Turn CCW (left track backward, right forward)
    setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, -motorSpeed);
    setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, motorSpeed);
  } else {
    // Turn CW (left track forward, right backward)
    setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, motorSpeed);
    setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, -motorSpeed);
  }
}

void stopHeadingControl() {
  headingControlActive = false;
  stopAllMotors();
  compassReadInterval = 100;  // Return to normal 10 Hz update rate
}

float calculateHeadingError(float current, float target) {
  float error = target - current;

  // Normalize to [-180, 180] range (shortest path)
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }

  return error;
}

// ===== COMPASS FUNCTIONS =====

float readCompassHeading() {
  int16_t x, y, z;
  mag.getHeading(&x, &y, &z);

  // Store raw values for debugging
  rawMagX = x;
  rawMagY = y;
  rawMagZ = z;

  // Apply hard iron offset correction
  x -= offsetX;
  y -= offsetY;
  z -= offsetZ;

  // Calculate heading (0-360 degrees)
  float heading = atan2((float)y, (float)x) * 180.0 / PI;
  if (heading < 0) heading += 360.0;

  return heading;
}

void loadCalibration() {
  struct CalibrationData {
    uint16_t magic;
    int16_t offsetX;
    int16_t offsetY;
    int16_t offsetZ;
  } cal;

  EEPROM.get(EEPROM_CALIB_ADDR, cal);

  if (cal.magic == CALIB_MAGIC) {
    offsetX = cal.offsetX;
    offsetY = cal.offsetY;
    offsetZ = cal.offsetZ;
    compassCalibrated = true;
    Serial.print("Calibration loaded: X=");
    Serial.print(offsetX);
    Serial.print(" Y=");
    Serial.print(offsetY);
    Serial.print(" Z=");
    Serial.println(offsetZ);
  } else {
    Serial.println("No calibration found - use calibrate_compass command");
    compassCalibrated = false;
  }
}

void saveCalibration() {
  struct CalibrationData {
    uint16_t magic;
    int16_t offsetX;
    int16_t offsetY;
    int16_t offsetZ;
  } cal;

  cal.magic = CALIB_MAGIC;
  cal.offsetX = offsetX;
  cal.offsetY = offsetY;
  cal.offsetZ = offsetZ;

  EEPROM.put(EEPROM_CALIB_ADDR, cal);
  Serial.println("Calibration saved to EEPROM");
}

void performCompassCalibration(int durationSec) {
  Serial.println("Starting compass calibration - rotating robot 360 degrees");

  int16_t minX = 32767, maxX = -32768;
  int16_t minY = 32767, maxY = -32768;
  int16_t minZ = 32767, maxZ = -32768;

  unsigned long startTime = millis();
  unsigned long duration = durationSec * 1000;

  // Rotate robot slowly during calibration
  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, 80);
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, -80);

  while (millis() - startTime < duration) {
    int16_t x, y, z;
    mag.getHeading(&x, &y, &z);

    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;

    delay(50); // 20 Hz sampling
  }

  forceStopMotors();

  // Calculate hard iron offsets
  offsetX = (maxX + minX) / 2;
  offsetY = (maxY + minY) / 2;
  offsetZ = (maxZ + minZ) / 2;

  compassCalibrated = true;
  saveCalibration();

  Serial.print("Calibration complete - Offsets: X=");
  Serial.print(offsetX);
  Serial.print(" Y=");
  Serial.print(offsetY);
  Serial.print(" Z=");
  Serial.println(offsetZ);
}
