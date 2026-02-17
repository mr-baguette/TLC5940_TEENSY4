/*
 * TLC5940 Logic Level Test
 * 
 * This will help diagnose if logic levels are the problem
 */

#include "TLC5940_Teensy4.h"

TLC5940Teensy4 tlc;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("TLC5940 Logic Level Diagnostic");
  Serial.println("================================");
  Serial.println();
  
  // Report pin voltages
  Serial.println("Teensy 4.1 outputs 3.3V logic levels");
  Serial.println("TLC5940 needs 4.0V minimum for logic HIGH");
  Serial.println();
  Serial.println("Testing with different approaches...");
  Serial.println();
  
  // Test 1: Initialize normally
  Serial.println("Test 1: Normal initialization");
  tlc.begin(0);
  delay(100);
  
  // Test 2: Try to turn on all LEDs
  Serial.println("Test 2: Setting all channels to FULL (4095)");
  tlc.setAll(4095);
  tlc.update();
  delay(2000);
  
  // Test 3: Try individual channels
  Serial.println("Test 3: Testing channel 0");
  tlc.clear();
  tlc.set(0, 4095);
  tlc.update();
  delay(2000);
  
  // Test 4: Toggle BLANK manually
  Serial.println("Test 4: Manual BLANK toggle test");
  Serial.println("Connect LED to OUT0 and watch...");
  
  pinMode(2, OUTPUT);  // BLANK pin
  
  for (int i = 0; i < 10; i++) {
    digitalWrite(2, HIGH);  // BLANK high = LEDs off
    delay(100);
    digitalWrite(2, LOW);   // BLANK low = LEDs on
    delay(100);
  }
  
  Serial.println();
  Serial.println("If LED didn't respond to BLANK toggle:");
  Serial.println("→ LOGIC LEVEL MISMATCH confirmed!");
  Serial.println();
  Serial.println("Solutions:");
  Serial.println("1. Add 2.2k pull-up resistors to signal pins");
  Serial.println("2. Use level shifter chips");
  Serial.println("3. Power TLC5940 from 3.3V instead of 5V");
}

void loop() {
  // Continuous slow fade to make any partial operation visible
  static uint16_t value = 0;
  
  value += 10;
  if (value > 4095) value = 0;
  
  tlc.setAll(value);
  tlc.update();
  delay(10);
}
