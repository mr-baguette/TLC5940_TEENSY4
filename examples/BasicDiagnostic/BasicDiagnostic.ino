/*
 * Simple diagnostic - NO TLC5940 library
 * This will tell us if the problem is the library or hardware
 */

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give Serial time to connect
  
  Serial.println("=================================");
  Serial.println("Starting Diagnostic");
  Serial.println("=================================");
  Serial.println();
  
  // Test 1: Can we configure pins at all?
  Serial.println("Test 1: Configuring pins as OUTPUT...");
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.println("  ✓ Pins configured");
  
  // Test 2: Can we write to them?
  Serial.println("Test 2: Writing HIGH to all pins...");
  digitalWrite(26, HIGH);
  digitalWrite(27, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(5, HIGH);
  Serial.println("  ✓ Writes successful");
  delay(500);
  
  // Test 3: Can we use OUTPUT_OPENDRAIN?
  Serial.println("Test 3: Trying OUTPUT_OPENDRAIN mode...");
  pinMode(26, OUTPUT_OPENDRAIN);
  pinMode(27, OUTPUT_OPENDRAIN);
  pinMode(3, OUTPUT_OPENDRAIN);
  pinMode(2, OUTPUT_OPENDRAIN);
  pinMode(5, OUTPUT_OPENDRAIN);
  Serial.println("  ✓ Open-drain mode set");
  delay(500);
  
  // Test 4: Can we initialize SPI?
  Serial.println("Test 4: Initializing SPI1...");
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  SPI1.begin();
  Serial.println("  ✓ SPI1 initialized");
  delay(500);
  
  // Test 5: Can we use analogWrite?
  Serial.println("Test 5: Testing analogWrite on pin 5...");
  analogWriteFrequency(5, 4000000);
  analogWrite(5, 128);
  Serial.println("  ✓ analogWrite working");
  delay(500);
  
  Serial.println();
  Serial.println("=================================");
  Serial.println("ALL TESTS PASSED!");
  Serial.println("=================================");
  Serial.println();
  Serial.println("If you see this, the Teensy is working.");
  Serial.println("Problem is likely in TLC5940 library initialization.");
  Serial.println();
  Serial.println("Measure voltages at Teensy pins:");
  Serial.println("  Pin 5 should show ~2.5V (PWM at 50%)");
  Serial.println("  Pins 2,3,26,27 should be pulled HIGH by pull-ups");
}

void loop() {
  // Blink LED to show we're alive
  static bool state = false;
  digitalWrite(LED_BUILTIN, state);
  state = !state;
  delay(500);
  
  Serial.print(".");
}
