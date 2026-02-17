#include <TLC5940_Teensy4.h>

TLC5940Teensy4 tlc;

#include <math.h>

#define NUM_CHANNELS 16
#define MAX_BRIGHTNESS 4095  // adjust to your desired max


float phase[NUM_CHANNELS]; // phase offset for each channel
float phaseStep = 0.015;    // speed of animation

void setup() {
  Serial.begin(115200);
  tlc.begin();
  
  // Initialize phase offsets evenly for wave effect
  for (int i = 0; i < NUM_CHANNELS; i++) {
    phase[i] = i * (TWO_PI / NUM_CHANNELS);
  }
}

void loop() {

  for (int i = 0; i < NUM_CHANNELS; i++) {
    float value = abs(sin(phase[i])) * MAX_BRIGHTNESS;

      value = MAX_BRIGHTNESS - value;

    tlc.set(i, (uint16_t)value);
    phase[i] += phaseStep;
    if (phase[i] > TWO_PI) phase[i] -= TWO_PI;
  }

  tlc.update();
  delay(10);
}
