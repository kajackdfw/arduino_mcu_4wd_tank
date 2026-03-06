#include <ArduinoJson.h>

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

// Wheel Pulse Sensors (FET triggered by wheel magnets)
// Pins 2 and 3 are INT0 and INT1 - the only hardware interrupt pins on ATmega32U4
#define LEFT_WHEEL_SENSOR  2  // Left wheel pulse sensor (INT0)
#define RIGHT_WHEEL_SENSOR 3  // Right wheel pulse sensor (INT1)
// Adjust RISING/FALLING to match sensor output polarity

// Safety state
bool emergencyStop = false;
bool leftCollision = false;
bool rightCollision = false;

// Wheel odometry pulse counts (volatile - modified by ISRs)
volatile unsigned long leftPulseCount = 0;
volatile unsigned long rightPulseCount = 0;

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

// Click-based movement control
bool clickBasedMovement = false;
unsigned long leftClickTarget = 0;
unsigned long rightClickTarget = 0;
unsigned long leftClickStart = 0;
unsigned long rightClickStart = 0;

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

  // Initialize wheel pulse sensors with interrupts
  pinMode(LEFT_WHEEL_SENSOR, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_SENSOR), leftWheelISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_SENSOR), rightWheelISR, RISING);

  // Start with motors stopped
  stopAllMotors();

  Serial.println("Tank Rover Ready");
  Serial.println("{\"cr_usb_device\":\"arduino_mcu_4wd_tank\"}");
}

void loop() {
  // Check safety switches first
  checkSafetySwitches();

  // Handle collision recovery (highest priority after safety)
  if (performingRecovery) {
    if (millis() - recoveryStartTime >= inverse_action_time) {
      performingRecovery = false;
      forceStopMotors();
      Serial.println("Recovery complete");
    }
    return; // Don't process commands during recovery
  }

  // Handle timed movement
  if (timedMovement) {
    if (millis() - movementStartTime >= movementDuration) {
      timedMovement = false;
      stopAllMotors();
      Serial.println("Timed movement complete");
    }
  }

  // Handle click-based movement
  if (clickBasedMovement) {
    noInterrupts();
    unsigned long l = leftPulseCount;
    unsigned long r = rightPulseCount;
    interrupts();
    if ((l - leftClickStart) >= leftClickTarget || (r - rightClickStart) >= rightClickTarget) {
      clickBasedMovement = false;
      stopAllMotors();
      Serial.println("Click target reached");
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

      if (doc.containsKey("clicks")) {
        startClickBasedMovement((unsigned long)doc["clicks"], (unsigned long)doc["clicks"]);
      } else if (duration > 0) {
        startTimedMovement(duration);
      } else {
        startTimedMovement(3000);
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
        int leftSpeed = doc.containsKey("left_speed") ? doc["left_speed"] : 150;
        int rightSpeed = doc.containsKey("right_speed") ? doc["right_speed"] : 150;
        setTrackSpeeds(leftSpeed, rightSpeed);
        if (doc.containsKey("left_click") || doc.containsKey("right_click")) {
          unsigned long lc = doc.containsKey("left_click")  ? (unsigned long)doc["left_click"]  : 0xFFFFFFFFUL;
          unsigned long rc = doc.containsKey("right_click") ? (unsigned long)doc["right_click"] : 0xFFFFFFFFUL;
          startClickBasedMovement(lc, rc);
        } else if (duration > 0) {
          startTimedMovement(duration);
        } else {
          startTimedMovement(3000);
        }
        Serial.println("OK");
        return;
      } else if (cmd == "stop") {
        stopAllMotors();
        timedMovement = false;
        clickBasedMovement = false;
        Serial.println("OK");
        return;
      } else if (cmd == "odometry") {
        noInterrupts();
        unsigned long l = leftPulseCount;
        unsigned long r = rightPulseCount;
        interrupts();
        Serial.print("{\"left_pulses\":");
        Serial.print(l);
        Serial.print(",\"right_pulses\":");
        Serial.print(r);
        Serial.println("}");
        return;
      } else if (cmd == "reset_odometry") {
        noInterrupts();
        leftPulseCount = 0;
        rightPulseCount = 0;
        interrupts();
        Serial.println("OK - Odometry reset");
        return;
      } else if (cmd == "identify") {
        Serial.println("{\"cr_usb_device\":\"arduino_mcu_4wd_tank\"}");
        return;
      } else {
        Serial.println("Unknown command");
        return;
      }

      if (doc.containsKey("clicks")) {
        startClickBasedMovement((unsigned long)doc["clicks"], (unsigned long)doc["clicks"]);
      } else if (duration > 0) {
        startTimedMovement(duration);
      } else {
        startTimedMovement(3000);
      }

      Serial.println("OK");
    }
  }
}

void setTrackSpeeds(int leftSpeed, int rightSpeed) {
  if (emergencyStop) {
    Serial.println("ERROR: Emergency stop active");
    return;
  }

  if (leftCollision || rightCollision) {
    Serial.println("ERROR: Collision detected, send {\"cmd\":\"reset\"} to clear");
    return;
  }

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, leftSpeed);
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, rightSpeed);
}

void setMotorPair(int enaPin, int in1Pin, int in2Pin, int enbPin, int in3Pin, int in4Pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
    analogWrite(enaPin, speed);
    analogWrite(enbPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
    analogWrite(enaPin, abs(speed));
    analogWrite(enbPin, abs(speed));
  } else {
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
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enablePin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enablePin, abs(speed));
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enablePin, 0);
  }
}

void stopAllMotors() {
  setTrackSpeeds(0, 0);
}

void checkSafetySwitches() {
  if (digitalRead(ESTOP_SWITCH) == LOW) {
    if (!emergencyStop) {
      emergencyStop = true;
      forceStopMotors();
      Serial.println("EMERGENCY STOP ACTIVATED");
    }
    return;
  } else {
    if (emergencyStop) {
      emergencyStop = false;
      Serial.println("Emergency stop cleared");
    }
  }

  if (digitalRead(LEFT_BUMPER) == LOW && !leftCollision) {
    leftCollision = true;
    Serial.println("LEFT COLLISION DETECTED - Reversing");
    performCollisionRecovery();
  }

  if (digitalRead(RIGHT_BUMPER) == LOW && !rightCollision) {
    rightCollision = true;
    Serial.println("RIGHT COLLISION DETECTED - Reversing");
    performCollisionRecovery();
  }
}

void performCollisionRecovery() {
  int inverseLeft = -currentLeftSpeed;
  int inverseRight = -currentRightSpeed;

  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, inverseLeft);
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, inverseRight);

  performingRecovery = true;
  recoveryStartTime = millis();
}

void forceStopMotors() {
  setMotorPair(LEFT_ENA, LEFT_IN1, LEFT_IN2, LEFT_ENB, LEFT_IN3, LEFT_IN4, 0);
  setMotorPair(RIGHT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, 0);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

void startTimedMovement(unsigned long duration) {
  clickBasedMovement = false;
  timedMovement = true;
  movementStartTime = millis();
  movementDuration = duration;
}

void startClickBasedMovement(unsigned long leftTarget, unsigned long rightTarget) {
  noInterrupts();
  leftClickStart = leftPulseCount;
  rightClickStart = rightPulseCount;
  interrupts();
  leftClickTarget = leftTarget;
  rightClickTarget = rightTarget;
  timedMovement = false;
  clickBasedMovement = true;
}

// ===== WHEEL ODOMETRY ISRs =====

void leftWheelISR() {
  leftPulseCount++;
}

void rightWheelISR() {
  rightPulseCount++;
}
