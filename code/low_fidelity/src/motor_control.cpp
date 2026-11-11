#include "motor_control.h"
#include <PinChangeInterrupt.h>

// =====================================================
// Motor driver pins
// =====================================================
const uint8_t IN1 = 5;
const uint8_t IN2 = 6;
const uint8_t IN3 = 9;
const uint8_t IN4 = 10;

const uint8_t ENA = 3;    // PWM
const uint8_t ENB = 11;   // PWM

// =====================================================
// Encoder pins
// =====================================================
const uint8_t ENC_A_M1 = 13;
const uint8_t ENC_B_M1 = 12;

const uint8_t ENC_A_M2 = 2;
const uint8_t ENC_B_M2 = 4;

// =====================================================
// Encoder / motion constants
// =====================================================
const long COUNTS_PER_REV_OUTPUT = 2800;
const unsigned long RPM_INTERVAL_MS = 100;

const int MIN_DRIVE_PWM   = 70;
const int MEDIUM_DRIVE_PWM = 110;
const int FAST_DRIVE_PWM   = 180;

const long POSITION_TOLERANCE_COUNTS = 4;

// =====================================================
// Encoder variables
// =====================================================
static volatile long encoderCountM1 = 0;
static volatile long encoderCountM2 = 0;

static volatile uint8_t lastStateM1 = 0;
static volatile uint8_t lastStateM2 = 0;

// RPM tracking
static unsigned long lastRPMTime = 0;
static long lastCountM1 = 0;
static long lastCountM2 = 0;
float rpmM1 = 0.0;
float rpmM2 = 0.0;

// =====================================================
// Encoder ISRs (file-local)
// =====================================================
static void encoderISR_M1() {
  uint8_t currentState = (digitalRead(ENC_A_M1) << 1) | digitalRead(ENC_B_M1);
  uint8_t transition = (lastStateM1 << 2) | currentState;

  if (transition == 0b0001 || transition == 0b0111 ||
      transition == 0b1110 || transition == 0b1000) {
    encoderCountM1++;
  }
  else if (transition == 0b0010 || transition == 0b0100 ||
           transition == 0b1101 || transition == 0b1011) {
    encoderCountM1--;
  }

  lastStateM1 = currentState;
}

static void encoderISR_M2() {
  uint8_t currentState = (digitalRead(ENC_A_M2) << 1) | digitalRead(ENC_B_M2);
  uint8_t transition = (lastStateM2 << 2) | currentState;

  if (transition == 0b0001 || transition == 0b0111 ||
      transition == 0b1110 || transition == 0b1000) {
    encoderCountM2++;
  }
  else if (transition == 0b0010 || transition == 0b0100 ||
           transition == 0b1101 || transition == 0b1011) {
    encoderCountM2--;
  }

  lastStateM2 = currentState;
}

// =====================================================
// Helper: choose slower PWM near target
// =====================================================
static int computeApproachPWM(long remainingCounts) {
  if (remainingCounts > 300) return FAST_DRIVE_PWM;
  if (remainingCounts > 80)  return MEDIUM_DRIVE_PWM;
  return MIN_DRIVE_PWM;
}

// =====================================================
// Initialization - call from setup()
// =====================================================
void motorControlSetup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoder pins
  pinMode(ENC_A_M1, INPUT_PULLUP);
  pinMode(ENC_B_M1, INPUT_PULLUP);
  pinMode(ENC_A_M2, INPUT_PULLUP);
  pinMode(ENC_B_M2, INPUT_PULLUP);

  // Read initial encoder states
  lastStateM1 = (digitalRead(ENC_A_M1) << 1) | digitalRead(ENC_B_M1);
  lastStateM2 = (digitalRead(ENC_A_M2) << 1) | digitalRead(ENC_B_M2);

  // Attach interrupts
  attachPCINT(digitalPinToPCINT(ENC_A_M1), encoderISR_M1, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_B_M1), encoderISR_M1, CHANGE);

  attachPCINT(digitalPinToPCINT(ENC_A_M2), encoderISR_M2, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_B_M2), encoderISR_M2, CHANGE);

  stopMotors();
  resetEncoders();
  resetRPMTracking();
}

// =====================================================
// Motor control
// =====================================================
void setMotorA(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  // Polarity flipped so positive PWM spins in the desired forward direction.
  if (speedValue > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speedValue);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setMotorB(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  // Polarity set to match Motor A and the desired forward direction.
  if (speedValue > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speedValue);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotorA() {
  setMotorA(0);
}

void stopMotorB() {
  setMotorB(0);
}

void stopMotors() {
  stopMotorA();
  stopMotorB();
}

// =====================================================
// Encoder helpers
// =====================================================
void resetEncoderM1() {
  noInterrupts();
  encoderCountM1 = 0;
  interrupts();
}

void resetEncoderM2() {
  noInterrupts();
  encoderCountM2 = 0;
  interrupts();
}

void resetEncoders() {
  resetEncoderM1();
  resetEncoderM2();
}

long getEncoderCountM1() {
  noInterrupts();
  long count = encoderCountM1;
  interrupts();
  return count;
}

long getEncoderCountM2() {
  noInterrupts();
  long count = encoderCountM2;
  interrupts();
  return count;
}

float getRevolutionsM1() {
  return (float)getEncoderCountM1() / (float)COUNTS_PER_REV_OUTPUT;
}

float getRevolutionsM2() {
  return (float)getEncoderCountM2() / (float)COUNTS_PER_REV_OUTPUT;
}

// =====================================================
// RPM helpers
// =====================================================
void resetRPMTracking() {
  lastRPMTime = millis();
  lastCountM1 = getEncoderCountM1();
  lastCountM2 = getEncoderCountM2();
  rpmM1 = 0.0;
  rpmM2 = 0.0;
}

void updateRPM() {
  unsigned long now = millis();

  if (now - lastRPMTime >= RPM_INTERVAL_MS) {
    long currentCountM1 = getEncoderCountM1();
    long currentCountM2 = getEncoderCountM2();

    long deltaM1 = currentCountM1 - lastCountM1;
    long deltaM2 = currentCountM2 - lastCountM2;

    rpmM1 = (deltaM1 * 60000.0f) / (COUNTS_PER_REV_OUTPUT * RPM_INTERVAL_MS);
    rpmM2 = (deltaM2 * 60000.0f) / (COUNTS_PER_REV_OUTPUT * RPM_INTERVAL_MS);

    lastCountM1 = currentCountM1;
    lastCountM2 = currentCountM2;
    lastRPMTime = now;
  }
}

void printMotorStatus() {
  updateRPM();

  Serial.print("M1 Count: ");
  Serial.print(getEncoderCountM1());
  Serial.print(" | M1 Rev: ");
  Serial.print(getRevolutionsM1(), 3);
  Serial.print(" | M1 RPM: ");
  Serial.print(rpmM1, 2);

  Serial.print(" || M2 Count: ");
  Serial.print(getEncoderCountM2());
  Serial.print(" | M2 Rev: ");
  Serial.print(getRevolutionsM2(), 3);
  Serial.print(" | M2 RPM: ");
  Serial.println(rpmM2, 2);
}

// =====================================================
// Time-based movement
// =====================================================
void moveMotorAForTime(int speedValue, float seconds) {
  Serial.print("Motor A time move: ");
  Serial.print(seconds, 2);
  Serial.println(" s");

  unsigned long durationMs = (unsigned long)(seconds * 1000.0f);
  unsigned long startTime = millis();

  resetRPMTracking();
  setMotorA(speedValue);

  while (millis() - startTime < durationMs) {
    printMotorStatus();
    delay(100);
  }

  stopMotorA();
  resetRPMTracking();
  Serial.println("Motor A time move complete.");
  printMotorStatus();
}

void moveMotorBForTime(int speedValue, float seconds) {
  Serial.print("Motor B time move: ");
  Serial.print(seconds, 2);
  Serial.println(" s");

  unsigned long durationMs = (unsigned long)(seconds * 1000.0f);
  unsigned long startTime = millis();

  resetRPMTracking();
  setMotorB(speedValue);

  while (millis() - startTime < durationMs) {
    printMotorStatus();
    delay(100);
  }

  stopMotorB();
  resetRPMTracking();
  Serial.println("Motor B time move complete.");
  printMotorStatus();
}

// =====================================================
// Position-based movement: revolutions
// =====================================================
void moveMotorAForRevolutions(float revolutions) {
  long targetCounts = lround(fabs(revolutions) * COUNTS_PER_REV_OUTPUT);
  int direction = (revolutions >= 0.0f) ? 1 : -1;

  resetEncoderM1();
  resetRPMTracking();

  Serial.print("Motor A target rev: ");
  Serial.print(revolutions, 3);
  Serial.print(" | Target counts: ");
  Serial.println(targetCounts);

  while (true) {
    long currentCounts = labs(getEncoderCountM1());
    long remaining = targetCounts - currentCounts;

    if (remaining <= POSITION_TOLERANCE_COUNTS) {
      break;
    }

    int pwm = computeApproachPWM(remaining);
    setMotorA(direction * pwm);

    updateRPM();

    Serial.print("Motor A Target: ");
    Serial.print(targetCounts);
    Serial.print(" | Current: ");
    Serial.print(currentCounts);
    Serial.print(" | Remaining: ");
    Serial.print(remaining);
    Serial.print(" | Rev: ");
    Serial.print((float)currentCounts / COUNTS_PER_REV_OUTPUT, 3);
    Serial.print(" | RPM: ");
    Serial.println(rpmM1, 2);

    delay(10);
  }

  stopMotorA();
  delay(50);
  stopMotorA();
  resetRPMTracking();

  long finalCounts = labs(getEncoderCountM1());

  Serial.print("Motor A done. Final counts: ");
  Serial.print(finalCounts);
  Serial.print(" | Final rev: ");
  Serial.println((float)finalCounts / COUNTS_PER_REV_OUTPUT, 3);
}

void moveMotorBForRevolutions(float revolutions) {
  long targetCounts = lround(fabs(revolutions) * COUNTS_PER_REV_OUTPUT);
  int direction = (revolutions >= 0.0f) ? 1 : -1;

  resetEncoderM2();
  resetRPMTracking();

  Serial.print("Motor B target rev: ");
  Serial.print(revolutions, 3);
  Serial.print(" | Target counts: ");
  Serial.println(targetCounts);

  while (true) {
    long currentCounts = labs(getEncoderCountM2());
    long remaining = targetCounts - currentCounts;

    if (remaining <= POSITION_TOLERANCE_COUNTS) {
      break;
    }

    int pwm = computeApproachPWM(remaining);
    setMotorB(direction * pwm);

    updateRPM();

    Serial.print("Motor B Target: ");
    Serial.print(targetCounts);
    Serial.print(" | Current: ");
    Serial.print(currentCounts);
    Serial.print(" | Remaining: ");
    Serial.print(remaining);
    Serial.print(" | Rev: ");
    Serial.print((float)currentCounts / COUNTS_PER_REV_OUTPUT, 3);
    Serial.print(" | RPM: ");
    Serial.println(rpmM2, 2);

    delay(10);
  }

  stopMotorB();
  delay(50);
  stopMotorB();
  resetRPMTracking();

  long finalCounts = labs(getEncoderCountM2());

  Serial.print("Motor B done. Final counts: ");
  Serial.print(finalCounts);
  Serial.print(" | Final rev: ");
  Serial.println((float)finalCounts / COUNTS_PER_REV_OUTPUT, 3);
}

// =====================================================
// Position-based movement: degrees
// =====================================================
void moveMotorAForDegrees(float degrees) {
  moveMotorAForRevolutions(degrees / 360.0f);
}

void moveMotorBForDegrees(float degrees) {
  moveMotorBForRevolutions(degrees / 360.0f);
}
