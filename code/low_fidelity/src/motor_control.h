#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// =====================================================
// Motor driver pins
// =====================================================
extern const uint8_t IN1;
extern const uint8_t IN2;
extern const uint8_t IN3;
extern const uint8_t IN4;
extern const uint8_t ENA;
extern const uint8_t ENB;

// =====================================================
// Encoder pins
// =====================================================
extern const uint8_t ENC_A_M1;
extern const uint8_t ENC_B_M1;
extern const uint8_t ENC_A_M2;
extern const uint8_t ENC_B_M2;

// =====================================================
// Encoder / motion constants
// =====================================================
extern const long COUNTS_PER_REV_OUTPUT;
extern const unsigned long RPM_INTERVAL_MS;

extern const int MIN_DRIVE_PWM;
extern const int MEDIUM_DRIVE_PWM;
extern const int FAST_DRIVE_PWM;

extern const long POSITION_TOLERANCE_COUNTS;

// =====================================================
// RPM data (read-only from outside)
// =====================================================
extern float rpmM1;
extern float rpmM2;

// =====================================================
// Initialization
// =====================================================
void motorControlSetup();

// =====================================================
// Motor control
// =====================================================
void setMotorA(int speedValue);
void setMotorB(int speedValue);
void stopMotorA();
void stopMotorB();
void stopMotors();

// =====================================================
// Encoder helpers
// =====================================================
void resetEncoderM1();
void resetEncoderM2();
void resetEncoders();

long getEncoderCountM1();
long getEncoderCountM2();

float getRevolutionsM1();
float getRevolutionsM2();

// =====================================================
// RPM helpers
// =====================================================
void resetRPMTracking();
void updateRPM();
void printMotorStatus();

// =====================================================
// Movement commands
// =====================================================
void moveMotorAForTime(int speedValue, float seconds);
void moveMotorBForTime(int speedValue, float seconds);

void moveMotorAForRevolutions(float revolutions);
void moveMotorBForRevolutions(float revolutions);

void moveMotorAForDegrees(float degrees);
void moveMotorBForDegrees(float degrees);

#endif // MOTOR_CONTROL_H
