#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// SERVO DECLARATIONS
// ============================================================================

Servo base;
Servo arm1;
Servo arm2;

const int BASE_PIN = 2;
const int ARM1_PIN = 3;
const int ARM2_PIN = 4;

// ============================================================================
// GRIPPER PINS
// ============================================================================

const int GRIPPER_ENABLE   = 9;
const int GRIPPER_A1       = 10;
const int GRIPPER_A2       = 11;
const int LIM_SWITCH_CLOSE = 6;   // Pin 6: object detected — stops closing
const int LIM_SWITCH_OPEN  = 7;   // Pin 7: fully open — stops opening

// ============================================================================
// ANGLE LIMITS
// ============================================================================

const int BASE_MIN = 110;
const int BASE_MAX = 180;
const int ARM1_MIN = 90;
const int ARM1_MAX = 140;
const int ARM2_MIN = 85;
const int ARM2_MAX = 130;

// ============================================================================
// GRIPPER STATE MACHINE
// ============================================================================

enum GripperState {
  GRIPPER_IDLE,
  GRIPPER_CLOSING,
  GRIPPER_OPENING
};

GripperState gripperState     = GRIPPER_IDLE;
unsigned long gripperStartTime = 0;
const unsigned long CLOSE_TIMEOUT = 5000; // failsafe if nothing grabbed

bool fullyOpen   = false;
bool fullyClosed = false;

// ============================================================================
// SERVO STATE
// ============================================================================

int basePos = 145;
int arm1Pos = 105;
int arm2Pos = 105;

int lastGripperCmd = 1; // 1=open, 0=close

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void startCloseGripper();
void startOpenGripper();
void updateGripper();
void motorClose();
void motorOpen();
void stopMotor();
void moveServos(int targetBase, int targetArm1, int targetArm2, int speed);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);

  pinMode(GRIPPER_ENABLE,   OUTPUT);
  pinMode(GRIPPER_A1,       OUTPUT);
  pinMode(GRIPPER_A2,       OUTPUT);
  pinMode(LIM_SWITCH_CLOSE, INPUT_PULLUP);
  pinMode(LIM_SWITCH_OPEN,  INPUT_PULLUP);

  stopMotor();

  delay(1000);

  base.attach(BASE_PIN);
  arm1.attach(ARM1_PIN);
  arm2.attach(ARM2_PIN);

  delay(500);

  base.write(basePos);
  arm1.write(arm1Pos);
  arm2.write(arm2Pos);

  // Home gripper to open on startup (non-blocking version starts it,
  // updateGripper() in loop() will finish it)
  Serial.println("Homing gripper to open...");
  startOpenGripper();

  Serial.println("Robot Arm Ready");
  Serial.println("Format: base,arm1,arm2,gripper,speed");
  Serial.println("Gripper: 0=close, 1=open");
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void motorClose() {
  digitalWrite(GRIPPER_ENABLE, HIGH);
  digitalWrite(GRIPPER_A1, LOW);
  digitalWrite(GRIPPER_A2, HIGH);
}

void motorOpen() {
  digitalWrite(GRIPPER_ENABLE, HIGH);
  digitalWrite(GRIPPER_A1, HIGH);
  digitalWrite(GRIPPER_A2, LOW);
}

void stopMotor() {
  digitalWrite(GRIPPER_ENABLE, LOW);
  digitalWrite(GRIPPER_A1, LOW);
  digitalWrite(GRIPPER_A2, LOW);
}

// ============================================================================
// GRIPPER — NON-BLOCKING START FUNCTIONS
// ============================================================================

void startCloseGripper() {
  if (!fullyOpen) {
    Serial.println("Cannot close: not fully open.");
    return;
  }
  Serial.println("Closing gripper...");
  fullyOpen        = false;
  fullyClosed      = false;
  gripperState     = GRIPPER_CLOSING;
  gripperStartTime = millis();
  motorClose();
}

void startOpenGripper() {
  if (!fullyClosed && fullyOpen) {
    Serial.println("Cannot open: not fully closed.");
    return;
  }
  Serial.println("Opening gripper...");
  fullyClosed      = false;
  fullyOpen        = false;
  gripperState     = GRIPPER_OPENING;
  gripperStartTime = millis();
  motorOpen();
}

// ============================================================================
// GRIPPER — NON-BLOCKING UPDATE (call every loop)
// ============================================================================

void updateGripper() {
  if (gripperState == GRIPPER_CLOSING) {
    // Stop on pin 6 (object grabbed)
    if (digitalRead(LIM_SWITCH_CLOSE) == LOW) {
      stopMotor();
      fullyClosed  = true;
      gripperState = GRIPPER_IDLE;
      Serial.println("Object grabbed (pin 6) — ready to open.");
      return;
    }
    // Stop on 5000ms failsafe
    if (millis() - gripperStartTime >= CLOSE_TIMEOUT) {
      stopMotor();
      fullyClosed  = true;
      gripperState = GRIPPER_IDLE;
      Serial.println("Close timeout (nothing grabbed) — ready to open.");
    }
  }

  if (gripperState == GRIPPER_OPENING) {
    // Stop on pin 7 (fully open)
    if (digitalRead(LIM_SWITCH_OPEN) == LOW) {
      stopMotor();
      fullyOpen    = true;
      gripperState = GRIPPER_IDLE;
      Serial.println("Fully open (pin 7) — ready to close.");
    }
  }
}

// ============================================================================
// SERVO MOVEMENT — still blocking per step but very short delays
// ============================================================================

void moveServos(int targetBase, int targetArm1, int targetArm2, int speed) {
  int delayTime = map(speed, 1, 100, 50, 5);

  int maxSteps = max(
    abs(targetBase - basePos),
    max(abs(targetArm1 - arm1Pos), abs(targetArm2 - arm2Pos))
  );

  for (int step = 0; step <= maxSteps; step++) {
    base.write(map(step, 0, maxSteps, basePos, targetBase));
    arm1.write(map(step, 0, maxSteps, arm1Pos, targetArm1));
    arm2.write(map(step, 0, maxSteps, arm2Pos, targetArm2));

    // Check gripper on every step so it stays responsive while arm moves
    updateGripper();

    delay(delayTime);
  }

  basePos = targetBase;
  arm1Pos = targetArm1;
  arm2Pos = targetArm2;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Always update gripper state every loop tick
  updateGripper();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    int values[5];
    int index    = 0;
    int startPos = 0;

    for (int i = 0; i <= (int)command.length(); i++) {
      if (i == (int)command.length() || command.charAt(i) == ',') {
        if (index < 5) {
          values[index] = command.substring(startPos, i).toInt();
          index++;
          startPos = i + 1;
        }
      }
    }

    if (index == 5) {
      int targetBase    = constrain(values[0], BASE_MIN, BASE_MAX);
      int targetArm1    = constrain(values[1], ARM1_MIN, ARM1_MAX);
      int targetArm2    = constrain(values[2], ARM2_MIN, ARM2_MAX);
      int targetGripper = constrain(values[3], 0, 1);
      int speed         = constrain(values[4], 1, 100);

      Serial.print("Moving to: ");
      Serial.print(targetBase);  Serial.print(",");
      Serial.print(targetArm1);  Serial.print(",");
      Serial.print(targetArm2);  Serial.print(",");
      Serial.print(targetGripper == 1 ? "open" : "close");
      Serial.print(",");
      Serial.println(speed);

      // Arm moves while gripper update runs inside moveServos()
      moveServos(targetBase, targetArm1, targetArm2, speed);

      // Only trigger gripper on a command change
      if (targetGripper != lastGripperCmd) {
        lastGripperCmd = targetGripper;
        if (targetGripper == 0) {
          startCloseGripper();
        } else {
          startOpenGripper();
        }
      }

      Serial.println("Movement complete");

    } else {
      Serial.println("ERROR: Expected format: base,arm1,arm2,gripper,speed");
      Serial.println("       Gripper: 0=close, 1=open");
    }
  }
}