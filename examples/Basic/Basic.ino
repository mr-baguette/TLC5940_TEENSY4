#include <TLC5940_Teensy4.h>

TLC5940Teensy4 tlc;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial or timeout
  
  Serial.println("TLC5940 Teensy 4.1 Test");
  Serial.println("Initializing...");
  
  // Initialize the TLC5940
  tlc.begin(0);
  
#if TLC5940_VPRG_ENABLED
  // Set all dot correction values to maximum
  tlc.setAllDc(0x3F);
  tlc.updateDc();
  Serial.println("Dot correction set to maximum");
#endif
  
  // Start with all LEDs off
  tlc.setAll(0);
  tlc.update();
  
  Serial.println("Initialization complete!");
  Serial.println("Starting fade test...");
}

void loop() {
  // Fade all channels up
  for (int i = 0; i < 3; i++)
    {
    for (uint16_t value = 0; value <= 4095; value += 32) {
      tlc.setAll(value);
      tlc.update();
      delay(5);
    }
  
    // Hold at maximum
    delay(500);
  
    // Fade all channels down
    for (int32_t value = 4095; value > 0; value -= 32) {
      tlc.setAll(value);
      tlc.update();
      delay(5);
    }
  
    // Hold at minimum
    tlc.setAll(0);
    tlc.update();
    delay(500);
  }

  // Individual channel test - light up each channel sequentially
  Serial.println("Sequential channel test...");
  for (uint16_t ch = 0; ch < 16 * TLC5940_NUM_CHIPS; ch++) {
    tlc.clear();
    tlc.set(ch, 4095);
    tlc.update();
    delay(100);
  }
  
  tlc.clear();
  tlc.update();
  delay(1000);
}
