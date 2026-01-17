#include <ArduinoJson.h>

// Motor Pin Definitions
// Left Track L298N (controlling Motors 1 & 3)
#define LEFT_ENA 3    // Left front motor speed
#define LEFT_IN1 22   // Left front motor direction
#define LEFT_IN2 23   // Left front motor direction
#define LEFT_ENB 5    // Left rear motor speed
#define LEFT_IN3 24   // Left rear motor direction
#define LEFT_IN4 25   // Left rear motor direction

// Right Track L298N (controlling Motors 2 & 4)
#define RIGHT_ENA 6   // Right front motor speed
#define RIGHT_IN1 26  // Right front motor direction
#define RIGHT_IN2 27  // Right front motor direction
#define RIGHT_ENB 9   // Right rear motor speed
#define RIGHT_IN3 28  // Right rear motor direction
#define RIGHT_IN4 29  // Right rear motor direction

// Safety Switches
#define LEFT_BUMPER 10    // Left collision sensor (normally open)
#define RIGHT_BUMPER 11   // Right collision sensor (normally open)
#define ESTOP_SWITCH 12   // Emergency stop (normally closed)

// Safety state
bool emergencyStop = false;
bool leftCollision = false;
bool rightCollision = false;

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

void setup() {
  Serial.begin(115200);
  
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
  
  // Start with motors stopped
  stopAllMotors();
  
  Serial.println("Tank Rover Ready");
}

void loop() {
  // Check safety switches first
  checkSafetySwitches();
  
  // Handle collision recovery
  if (performingRecovery) {
    if (millis() - recoveryStartTime >= inverse_action_time) {
      // Recovery complete, stop motors
      performingRecovery = false;
      forceStopMotors(); // Direct stop, bypassing safety checks
      Serial.println("Recovery complete");
    }
    return; // Don't process commands during recovery
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
