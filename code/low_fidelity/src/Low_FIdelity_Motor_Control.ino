/*
  Low Fidelity Motor Control - Main
  ----------------------------------
  Senses input from sensors (e.g. MPU6050) and controls
  motors based on that input.

  Motor control logic lives in motor_control.h / .cpp

  Wiring:
  Motor A: IN1(5), IN2(6), ENA(3), Enc_A(13), Enc_B(12)
  Motor B: IN3(9), IN4(10), ENB(11), Enc_A(2), Enc_B(4)
*/

#include "motor_control.h"
#include "assist_control.h"

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Low Fidelity Motor Control Starting...");

  motorControlSetup();
  assistSetup();

  Serial.println("Setup complete.");
  Serial.println();
}

void loop() {
  // Power-assist control loop. assistUpdate() is non-blocking and paces
  // itself at LOOP_MS (20 ms / 50 Hz) internally using millis().
  assistUpdate();
}
