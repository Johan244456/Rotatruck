#ifndef ASSIST_CONTROL_H
#define ASSIST_CONTROL_H

#include <Arduino.h>

enum AssistState {
  STATE_IDLE,
  STATE_ASSISTING,
  STATE_STOPPING,
  STATE_ESTOP
};

// Call once from setup() after Serial.begin() and motorControlSetup().
// Initialises IMU, calibrates gravity offset, prepares PIDs.
void assistSetup();

// Call every loop() iteration. Internally paces at LOOP_MS using millis().
void assistUpdate();

AssistState assistGetState();
const char* assistStateName(AssistState s);

#endif // ASSIST_CONTROL_H
