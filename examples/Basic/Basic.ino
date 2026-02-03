#include <TLC5940_Teensy4.h>

TLC5940Teensy4 tlc;

void setup() {
  tlc.begin();
  tlc.setAll(0);
#if TLC5940_VPRG_ENABLED
  tlc.setAllDc(0x3F);
  tlc.updateDc();
#endif
  tlc.update();
}

void loop() {
  for (uint16_t value = 0; value <= 4095; value += 128) {
    tlc.setAll(value);
    tlc.update();
    tlc.pollXerr();
    delay(20);
  }
}
