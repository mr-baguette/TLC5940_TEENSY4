/*
 * TLC5940 Smooth Fade Test - Fixed Version
 * For TLC5940_Teensy4 Library
 * 
 * This demonstrates smooth fading without audible noise.
 * GSCLK is set to 30 MHz for silent operation.
 */

#include "TLC5940_Teensy4.h"

TLC5940Teensy4 tlc;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("TLC5940 Silent Fade Test");
  Serial.println("=========================");
  Serial.println("GSCLK: 30 MHz (for silent operation)");
  Serial.println("PWM Frequency: ~7.3 kHz (barely audible)");
  Serial.println();
  
  // Initialize TLC5940
  tlc.begin(0);  // Start with all channels off
  
  Serial.println("Initialization complete!");
  Serial.println("Starting continuous fade...");
  Serial.println();
}

void loop() {
  int16_t value;  // Use signed int to detect when we go below 0
  
  // Fade UP: 0 → 4095
  for (value = 0; value <= 4095; value += 4) {
    tlc.setAll(value);
    tlc.update();
    delayMicroseconds(500);  // Smooth visible fade
  }
  
  Serial.println("↑ Fade UP complete (0 → 4095)");
  delay(250);
  
  // Fade DOWN: 4095 → 0
  // Fixed: Use signed int and check >= 0 to avoid underflow
  for (value = 4095; value >= 0; value -= 4) {
    tlc.setAll(value);
    tlc.update();
    delayMicroseconds(500);
  }
  
  // Ensure we end at exactly 0
  tlc.setAll(0);
  tlc.update();
  
  Serial.println("↓ Fade DOWN complete (4095 → 0)");
  delay(250);
}
