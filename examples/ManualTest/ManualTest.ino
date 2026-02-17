/*
 * ABSOLUTE MINIMAL TLC5940 Test
 * No library - just raw pin manipulation
 * This will tell us if the chip is alive
 */

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for Serial to connect
  
  Serial.println("====================================");
  Serial.println("TLC5940 Manual Test - No Library");
  Serial.println("====================================");
  Serial.println();
  
  Serial.println("Step 1: Configuring pins...");
  pinMode(26, OUTPUT);  // SIN
  pinMode(27, OUTPUT);  // SCLK
  pinMode(3, OUTPUT);   // XLAT
  pinMode(2, OUTPUT);   // BLANK
  pinMode(5, OUTPUT);   // GSCLK
  pinMode(25, OUTPUT);  // VPRG
  pinMode(7, OUTPUT);   // DCPRG
  
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(3, LOW);
  digitalWrite(2, HIGH);  // BLANK high = outputs off
  digitalWrite(5, LOW);
  digitalWrite(25, LOW);  // VPRG low = grayscale mode
  digitalWrite(7, HIGH);  // DCPRG high (not used but set anyway)
  
  Serial.println("  ✓ Pins configured");
  Serial.println();
  
  // Check what we're working with
  Serial.println("Current Setup:");
  Serial.println("  TLC5940 VCC: 3.3V (you changed it)");
  Serial.println("  Teensy outputs: 3.3V");
  Serial.println("  Expected: Should work now!");
  Serial.println();
  
  delay(1000);
  
  // Test 1: Turn BLANK low (should turn on LEDs if data is there)
  Serial.println("Test 1: Setting BLANK LOW (enables outputs)");
  digitalWrite(2, LOW);
  delay(2000);
  Serial.println("  If LED didn't light, chip may not have data");
  Serial.println();
  
  digitalWrite(2, HIGH);  // Turn outputs off again
  delay(500);
  
  // Test 2: Send some data manually
  Serial.println("Test 2: Sending grayscale data (all channels MAX)...");
  Serial.println("  Sending 192 bytes (16 channels × 12 bits)");
  
  // Send all 1's (max brightness for all channels)
  for (int i = 0; i < 192; i++) {
    // Shift out a byte manually
    for (int bit = 7; bit >= 0; bit--) {
      digitalWrite(26, HIGH);  // Always send 1
      digitalWrite(27, HIGH);  // Clock high
      delayMicroseconds(1);
      digitalWrite(27, LOW);   // Clock low
      delayMicroseconds(1);
    }
  }
  
  Serial.println("  ✓ Data sent");
  Serial.println();
  
  // Test 3: Pulse XLAT to latch data
  Serial.println("Test 3: Pulsing XLAT (latch data)...");
  digitalWrite(3, HIGH);
  delayMicroseconds(2);
  digitalWrite(3, LOW);
  Serial.println("  ✓ XLAT pulsed");
  Serial.println();
  
  // Test 4: Turn on outputs
  Serial.println("Test 4: Setting BLANK LOW (turn on outputs)...");
  digitalWrite(2, LOW);
  Serial.println("  ✓ BLANK is LOW");
  Serial.println();
  
  // Test 5: Start GSCLK
  Serial.println("Test 5: Starting GSCLK (PWM clock)...");
  analogWriteFrequency(5, 100000);  // 100kHz for easy debugging
  analogWrite(5, 128);
  Serial.println("  ✓ GSCLK running at 100kHz");
  Serial.println();
  
  Serial.println("====================================");
  Serial.println("TESTS COMPLETE");
  Serial.println("====================================");
  Serial.println();
  Serial.println("*** CHECK YOUR LED NOW ***");
  Serial.println();
  Serial.println("LED on OUT0 should be:");
  Serial.println("  - Fully ON if everything works");
  Serial.println("  - Off if chip is not responding");
  Serial.println();
  Serial.println("If still OFF, check:");
  Serial.println("  1. Is VCC actually 3.3V? (measure with multimeter)");
  Serial.println("  2. Is GND connected?");
  Serial.println("  3. Is IREF resistor (2.2k) connected?");
  Serial.println("  4. Is LED connected correctly (cathode to OUT0)?");
  Serial.println("  5. Is chip orientation correct?");
  Serial.println();
  
  Serial.println("Voltages to measure:");
  Serial.println("  TLC Pin 21 (VCC): should be 3.3V");
  Serial.println("  TLC Pin 22 (GND): should be 0V");
  Serial.println("  TLC Pin 20 (IREF): should be ~1.5V");
  Serial.println("  TLC Pin 27 (VPRG): should be 0V");
  Serial.println("  TLC Pin 19 (DCPRG): should be 3.3V");
  Serial.println("  TLC Pin 23 (BLANK): should be 0V (outputs enabled)");
}

void loop() {
  // Blink onboard LED so we know Teensy is running
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (millis() - lastBlink > 500) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlink = millis();
    Serial.print(".");
  }
}
