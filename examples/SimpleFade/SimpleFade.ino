/*
 * Simple TLC5940 Test for Teensy 4.1
 * Using TLC5940_Teensy4 Library
 */

#include "TLC5940_Teensy4.h"

TLC5940Teensy4 tlc;
uint16_t value = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("TLC5940 Teensy 4.1 Simple Test");
  Serial.println("===============================");
  
  // Initialize TLC5940
  tlc.begin(0);  // Start with all channels off
  
  Serial.println("Initialization complete!");
  Serial.println("Starting fade test...");
}

void loop() {
  // Fade up
  for (value = 0; value <= 4095; value += 10) {
    tlc.setAll(value);
    tlc.update();
    delay(2);
  }
  
  Serial.println("Fade UP complete");
  delay(500);
  
  // Fade down
  for (value = 4095; value > 0; value -= 10) {
    tlc.setAll(value);
    tlc.update();
    delay(2);
  }
  
  Serial.println("Fade DOWN complete");
  delay(500);
}
