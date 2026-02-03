#ifndef TLC5940_TEENSY4_CONFIG_H
#define TLC5940_TEENSY4_CONFIG_H

// Configuration for TLC5940 Teensy 4.1 driver.
// Adjust these values to match your wiring and TLC5940 chain length.

#ifndef TLC5940_NUM_CHIPS
#define TLC5940_NUM_CHIPS 1
#endif

// Pin assignments (Teensy 4.1 defaults can be overridden in a sketch).
#ifndef TLC5940_PIN_SIN
#define TLC5940_PIN_SIN 11
#endif

#ifndef TLC5940_PIN_SCLK
#define TLC5940_PIN_SCLK 13
#endif

#ifndef TLC5940_PIN_XLAT
#define TLC5940_PIN_XLAT 10
#endif

#ifndef TLC5940_PIN_BLANK
#define TLC5940_PIN_BLANK 9
#endif

#ifndef TLC5940_PIN_GSCLK
#define TLC5940_PIN_GSCLK 8
#endif

#ifndef TLC5940_PIN_DCPRG
#define TLC5940_PIN_DCPRG 7
#endif

#ifndef TLC5940_PIN_VPRG
#define TLC5940_PIN_VPRG 6
#endif

// Enable hardware SPI for grayscale data transfer when possible.
#ifndef TLC5940_USE_SPI
#define TLC5940_USE_SPI 1
#endif

// SPI settings
#ifndef TLC5940_SPI_CLOCK
#define TLC5940_SPI_CLOCK 10000000
#endif

// Dot-correction support (requires VPRG wiring).
#ifndef TLC5940_VPRG_ENABLED
#define TLC5940_VPRG_ENABLED 1
#endif

// XERR handling.
#ifndef TLC5940_PIN_XERR
#define TLC5940_PIN_XERR 12
#endif

// Minimum interval between BLANK pulses used for XERR LED-open detection (microseconds).
#ifndef TLC5940_XERR_BLANK_INTERVAL_US
#define TLC5940_XERR_BLANK_INTERVAL_US 2000
#endif

// Duration of a BLANK pulse used for XERR LED-open detection (microseconds).
#ifndef TLC5940_XERR_BLANK_PULSE_US
#define TLC5940_XERR_BLANK_PULSE_US 2
#endif

#endif // TLC5940_TEENSY4_CONFIG_H
