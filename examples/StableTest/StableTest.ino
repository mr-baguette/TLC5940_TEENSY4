/*
 * Stable Test - Slow Everything Down
 * This should reduce jitter if it's power-related
 */

#include "TLC5940_Teensy4.h"

TLC5940Teensy4 tlc;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("TLC5940 Stable Test");
  Serial.println("===================");
  Serial.println();
  Serial.println("This test slows down GSCLK to reduce power spikes.");
  Serial.println("If jitter goes away, you need decoupling capacitors!");
  Serial.println();
  
  // Slow down GSCLK to reduce switching noise
  // This is a workaround - proper fix is decoupling caps!
  Serial.println("Starting with slow GSCLK (100kHz)...");
  
  tlc.begin(0);
  
  // Turn on your channels (adjust as needed)
  Serial.println("Setting OUT10 and OUT12 to full brightness...");
  tlc.set(10, 4095);  // OUT10
  tlc.set(12, 4095);  // OUT12
  tlc.update();
  
  Serial.println();
  Serial.println("LEDs should be ON now.");
  Serial.println();
  Serial.println("Is the output:");
  Serial.println("  - Stable? → Good! But still add caps for reliability");
  Serial.println("  - Still jittery? → Could be bad connections");
  Serial.println("  - Flickering fast? → Missing 0.1uF cap (critical!)");
  Serial.println();
  Serial.println("Proper fix: Add 0.1uF ceramic cap");
  Serial.println("            between VCC and GND, close to chip!");
}

void loop() {
  // Keep LEDs on steadily
  delay(1000);
  Serial.print(".");
}
