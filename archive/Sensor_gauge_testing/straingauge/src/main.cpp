#include <Arduino.h>
#include "HX711.h"
#include <Arduino.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
byte gain = 128;
HX711 gauge;
bool tare = false;

char cmd;

// put function declarations here:
int myFunction(int, int);

const float weights[] = {1.5, 3, 5, 7, 10, 13};
const int num_weights = sizeof(weights) / sizeof(weights[0]);

void wait_for_serial_input() {
  while (Serial.available() == 0) {
    delay(10); // wait for user input
  }
  while (Serial.available() > 0) {
    Serial.read(); // clear buffer
  }
}

void setup(){
Serial.begin(57600);
Serial.println("Initialising..");
gauge.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, gain);
Serial.print("Initialising... waiting 1000ms for thermal stability...");
delay(1000); 

// 2. Tare Sequence
Serial.print("\nTaring system. Ensure beam is unloaded...");
gauge.tare(50);
Serial.println(" Tare complete.");
// Serial.print("Initial read: \t \t ");
// Serial.println(gauge.read_average(20));
// for (int i = 0; i < num_weights; i++) {
//   Serial.print("\nPlace ");
//   Serial.print(weights[i], 1);
//   Serial.println(" kg weight on the scale and press any key to continue...");
//   wait_for_serial_input();
//   delay(500); // allow settling
//   long avg = gauge.read_average(20);
//   Serial.print("Average reading for ");
//   Serial.print(weights[i], 1);
//   Serial.print(" kg: ");
  // Serial.println(avg);
// }
// Serial.println("\nAll weights processed. Ready for further commands.");
}
void loop() {
  if (gauge.is_ready()) {
    // 1. Get the raw un-tared reading
    long current_reading = gauge.read(); 
    long current_offset = gauge.get_offset();
    
    // Define how much change is considered "drift" vs "an actual weight"
    // You may need to tune this number (500) based on your sensor's noise flooring
    long drift_threshold = 5000; 

    // 2. Auto-Zeroing / Tare Tracking
    // If the reading is close to our zero point (offset), it's just thermal drift
    if (abs(current_reading - current_offset) < drift_threshold) {
        // Slowly pull the offset towards the new empty reading (rolling average)
        current_offset = (current_offset * 3 + current_reading) / 4; 
        gauge.set_offset(current_offset);
    }
    
    // 3. Data Acquisition
    // Now get the value (which subtracts the newly adjusted offset automatically)
    Serial.print("Avg of 5 results: \t \t ");
    Serial.println(gauge.get_value(5)); 
    
  } else {
    // Optional: Yield a brief moment to prevent microcontroller lockup 
    // while waiting for the next sample window.
    delay(10);
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}